from pyocamcalib.modelling.camera import Camera
from pyrecipnps.recipnps.p3p import grunert, fischler, kneip
from pyrecipnps.recipnps.pnp import pnp_ransac_fischler, pnp_ransac_grunert, pnp_ransac_kneip
from pyrecipnps.recipnps.model import Model
import numpy as np
from collections.abc import Callable

class PnPSolver:
	def __init__(self, calib_path: str = None, uav_size: float = 0):
		if calib_path is None:
			raise ValueError("Calibration data must be provided.")
		
		if uav_size <= 0:
			raise ValueError("UAV size must be a positive value.")
		
		self.cam = Camera().load_parameters_json(calib_path)
		self.uav_size = uav_size
		self._object_points = np.array([
			[uav_size, 0, 0],			# LED1 (0)
			[0, 0, 0],					# LED2 (1)
			[uav_size, uav_size, 0],	# LED3 (2)
			[0, uav_size, 0]			# LED4 (3)
		], dtype=np.float32)
		
		self.method_p3p = fischler

		self.method_pnp = pnp_ransac_fischler
		self.max_iter = 10
		self.inlier_dist_threshold = 2.0
		self.probability = 0.99

	def _get_cam_rays(self, image_points):
		"""
			Converts the image pixels to the corresponding unit sphere rays, based on the camera calibration parameters.
		"""
		rays = self.cam.cam2world(np.array(image_points, dtype=np.float32))
		return rays
	
	def is_real(self, solution: Model):
		return np.allclose(solution.rotation.imag, 0) and np.allclose(solution.translation.imag, 0)

	def compute_reprojection_error(self, R, t, object_points, image_points):
		camera_points = (R @ object_points.T).T + t
		
		reprojected_points = self.cam.world2cam(camera_points)
		
		errors = np.linalg.norm(reprojected_points - image_points, axis=1)
		mean_error = np.mean(errors)
		
		return mean_error

	def _solve_p3p(self, image_points: np.ndarray, indices: np.ndarray, method: Callable = None, verbose: bool = True):
		rays = self._get_cam_rays(image_points).T
		rays = rays / np.linalg.norm(rays, axis=0)
		object_points = self._object_points[indices].T

		if method is None:
			method = self.method_p3p

		sol = method(object_points, rays)

		sol = [s for s in sol if self.is_real(s)]

		errs = np.array([self.compute_reprojection_error(np.real(s.rotation), np.real(s.translation), object_points, image_points) for s in sol])

		if verbose:
			print(f"Solving using the {method.__name__} method")
			print(f"==============")

			for idx, (s, err) in enumerate(zip(sol, errs)):
				print(f"-->")
				print(f"Solution {idx}:")
				print(f"Rotation: {np.real(s.rotation)}")
				print(f"Translation: {np.real(s.translation)}")
				print(f"Reprojection Error: {err}")

		#select the solution with the smallest reprojection error

		best_solution = sol[np.argmin(errs)]
		best_error = np.min(errs)

		distance = np.linalg.norm(np.real(best_solution.translation))

		res = {
			"rotation": np.real(best_solution.rotation),
			"translation": np.real(best_solution.translation),
			"reprojection_error": best_error
		}

		if verbose:
			print(f"-->")
			print(f"Best solution:")
			print(f"Rotation: {res['rotation']}")
			print(f"Translation: {res['translation']}")
			print(f"Distance: {distance}")
			print(f"Reprojection Error: {best_error}")

		return res
	
	def _solve_pnp(self, image_points: np.ndarray, indices: np.ndarray, method: Callable = None, verbose: bool = True, ransac_retries: int = 10):
		if len(indices) < 4:
			raise ValueError("RANSAC PnP requires at least 4 points.")

		# Get rays and object points
		rays = self._get_cam_rays(image_points).T
		rays = rays / np.linalg.norm(rays, axis=0)
		object_points = self._object_points[indices].T

		if method is None:
			method = self.method_pnp

		best_solution = None
		best_error = float('inf')

		for attempt in range(ransac_retries):
			# Run RANSAC
			sol = method(
				object_points,
				rays,
				max_iterations=self.max_iter,
				inlier_dist_threshold=self.inlier_dist_threshold,
				probability=self.probability
			)

			if sol is None:
				if verbose:
					print(f"RANSAC attempt {attempt + 1} failed.")
				continue

			R = np.real(sol.rotation)
			t = np.real(sol.translation)

			error = self.compute_reprojection_error(R, t, object_points.T, image_points)

			if error < best_error:
				best_error = error
				best_solution = {
					"rotation": R,
					"translation": t,
					"reprojection_error": error,
					"distance": np.linalg.norm(t)
				}

				if verbose:
					print(f"New best solution (attempt {attempt + 1}):")
					print(f"Error: {error:.2f} px | Distance: {best_solution['distance']:.2f} m")

		if best_solution is None:
			if verbose:
				print("All RANSAC attempts failed.")
			return None

		if best_error > 10.0:
			if verbose:
				print(f"Best error too high ({best_error:.2f} px). Rejecting.")
			return None

		if verbose:
			print("\n=== Final RANSAC Solution ===")
			print(f"Rotation:\n{best_solution['rotation']}")
			print(f"Translation:\n{best_solution['translation']}")
			print(f"Reprojection Error: {best_error:.2f} px")
			print(f"Distance: {best_solution['distance']:.2f} m")

		return best_solution
	
	def solve(self, image_points: np.ndarray, indices: np.ndarray, method_p3p: Callable = None, method_pnp: Callable = None, verbose: bool = True, ransac_retries: int = 10):
		num_points = len(image_points)

		if num_points < 3:
			raise ValueError("At least 3 points are required to solve PnP.")
		
		res = None

		if num_points >= 4:
			res = self._solve_pnp(image_points, indices, method=method_pnp, verbose=verbose, ransac_retries=ransac_retries)

		if num_points == 3:
			res = self._solve_p3p(image_points, indices, method=method_p3p, verbose=verbose)

		return res

if __name__ == "__main__":
	pnps = PnPSolver(calib_path="calibration.json", uav_size=425)

	led1 = [684, 111]
	led2 = [537, 110]
	led3 = [699, 179]
	led4 = [527, 179]

	points = [led1, led2, led3, led4]

	indices = [0, 1, 2, 3]

	# solution = pnps._solve_pnp(points, indices, verbose=True)

	# led1 = [734, 352]
	# led3 = [681, 424]
	# led4 = [525, 378]

	# points = [led1, led3, led4]
	
	# indices = [0, 2, 3]
	
	solution = pnps.solve(points, indices, verbose=True)