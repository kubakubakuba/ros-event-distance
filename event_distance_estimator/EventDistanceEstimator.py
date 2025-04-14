#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from event_distance_estimator.msg import LEDMarkers
from PnPSolver import PnPSolver
from tf.transformations import quaternion_from_matrix

class LEDDistanceEstimatorNode:
	def __init__(self):
		rospy.init_node("led_distance_estimator")
		
		self.pnp_solver = PnPSolver(calib_path=rospy.get_param('~calibration_file'), uav_size=425)

		self.led_sub = rospy.Subscriber("/detected_leds", LEDMarkers, self.led_callback)
		
		self.pose_pub = rospy.Publisher("/estimated_pose", PoseStamped, queue_size=1)
		self.distance_pub = rospy.Publisher("/estimated_distance", PointStamped, queue_size=1)
		
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
					distance = np.linalg.norm(position)
					self.publish_results(position, rotation_matrix, distance)
				else:
					rospy.logwarn("PnP solving returned no result.")

			except Exception as e:
				rospy.logwarn(f"PnP solving failed: {str(e)}")

	def rotation_matrix_to_quaternion(self, R):
		"""Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
		R_4x4 = np.eye(4)
		R_4x4[:3, :3] = R
		return quaternion_from_matrix(R_4x4)

	def publish_results(self, position, rotation_matrix, distance):
		# Convert position from mm to meters
		position_meters = [coord / 1000.0 for coord in position]
		distance_meters = distance / 1000.0

		quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
		
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
		
		distance_msg = PointStamped()
		distance_msg.header.stamp = rospy.Time.now()
		distance_msg.header.frame_id = "camera_frame"
		
		distance_msg.point.x = position_meters[0]
		distance_msg.point.y = position_meters[1]
		distance_msg.point.z = position_meters[2]
		
		self.distance_pub.publish(distance_msg)

		rospy.loginfo(f"Published pose: {position_meters}, distance: {distance_meters:.2f} m, with quaternion: {quaternion}")

if __name__ == '__main__':
	try:
		rospy.loginfo("Starting LED Distance Estimator")
		node = LEDDistanceEstimatorNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
