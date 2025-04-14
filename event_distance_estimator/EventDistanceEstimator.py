#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from event_distance_estimator.msg import LEDMarkers
from PnPSolver import PnPSolver
from tf.transformations import quaternion_from_matrix

class LEDDistanceEstimator:
	def __init__(self):
		rospy.init_node("led_distance_estimator")
		
		self.pnp_solver = PnPSolver(calib_path=rospy.get_param('~calibration_file'), uav_size=425)

		self.led_sub = rospy.Subscriber("/detected_leds", LEDMarkers, self.led_callback)
		
		self.pose_pub = rospy.Publisher("/estimated_pose", PoseStamped, queue_size=1)
		self.distance_pub = rospy.Publisher("/estimated_distance", Float32, queue_size=1)

		rospy.loginfo("LED Distance Estimator initialized")

	def led_callback(self, msg):
		points = []
		indices = []
		
		for point, led_id in zip(msg.points, msg.ids):
			points.append([point.x, point.y])
			indices.append(led_id)
		
		if len(points) >= 3:
			try:
				result = self.pnp_solver.solve(np.array(points), np.array(indices), verbose=False)
				
				if result is not None:
					position = result['translation']
					rotation_matrix = result['rotation']
					distance_mm = np.linalg.norm(position)
					distance_meters = distance_mm / 1000.0
					self.publish_results(position, rotation_matrix, distance_meters)
				else:
					rospy.logwarn("PnP solving returned no result.")

			except Exception as e:
				rospy.logwarn(f"PnP solving failed: {str(e)}")

	def rotation_matrix_to_quaternion(self, R):
		"""Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
		R_4x4 = np.eye(4)
		R_4x4[:3, :3] = R
		return quaternion_from_matrix(R_4x4)

	def publish_results(self, position_mm, rotation_matrix, distance_meters):
		# Convert position from mm to meters
		position_meters = [coord / 1000.0 for coord in position_mm]

		quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
		
		# Publish pose (in meters)
		pose_msg = PoseStamped()
		pose_msg.header.stamp = rospy.Time.now()
		pose_msg.header.frame_id = "camera_frame"
		pose_msg.pose.position.x = position_meters[0]
		pose_msg.pose.position.y = position_meters[1]
		pose_msg.pose.position.z = position_meters[2]
		pose_msg.pose.orientation.x = quaternion[0]
		pose_msg.pose.orientation.y = quaternion[1]
		pose_msg.pose.orientation.z = quaternion[2]
		pose_msg.pose.orientation.w = quaternion[3]
		self.pose_pub.publish(pose_msg)
		
		# Publish distance as simple float (in meters)
		distance_msg = Float32()
		distance_msg.data = distance_meters
		self.distance_pub.publish(distance_msg)

		rospy.loginfo(f"Published distance: {distance_meters:.2f} m")

if __name__ == '__main__':
	try:
		rospy.loginfo("Starting LED Distance Estimator")
		node = LEDDistanceEstimator()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass