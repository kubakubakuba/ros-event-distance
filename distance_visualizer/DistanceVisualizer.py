#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import tf.transformations as tf

class DistanceVisualizer:
	def __init__(self):
		rospy.init_node('uav_visualizer_rviz')
		
		self.pose_sub = rospy.Subscriber("/estimated_pose", PoseStamped, self.pose_callback)
		self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
		
		rospy.loginfo("UAV RViz Visualizer ready")

	def pose_callback(self, msg):
		uav_center_marker = Marker()
		uav_center_marker.header = msg.header
		uav_center_marker.ns = "estimated_uav"
		uav_center_marker.id = 0
		uav_center_marker.type = Marker.SPHERE
		uav_center_marker.action = Marker.ADD
		
		uav_center_marker.pose = msg.pose
		uav_center_marker.scale.x = 0.1
		uav_center_marker.scale.y = 0.1
		uav_center_marker.scale.z = 0.1
		
		uav_center_marker.color.r = 1.0
		uav_center_marker.color.g = 1.0
		uav_center_marker.color.b = 1.0
		uav_center_marker.color.a = 1.0
		
		uav_center_marker.lifetime = rospy.Duration()

		# Camera Marker
		camera_marker = Marker()
		camera_marker.header = msg.header
		camera_marker.ns = "camera"
		camera_marker.id = 1
		camera_marker.type = Marker.SPHERE
		camera_marker.action = Marker.ADD
		
		camera_marker.pose.position.x = 0.0
		camera_marker.pose.position.y = 0.0
		camera_marker.pose.position.z = 0.0
		camera_marker.pose.orientation.x = 0.0
		camera_marker.pose.orientation.y = 0.0
		camera_marker.pose.orientation.z = 0.0
		camera_marker.pose.orientation.w = 1.0
		
		camera_marker.scale.x = 0.1
		camera_marker.scale.y = 0.1
		camera_marker.scale.z = 0.1
		
		camera_marker.color.r = 0.0
		camera_marker.color.g = 0.0
		camera_marker.color.b = 1.0
		camera_marker.color.a = 1.0

		camera_marker.lifetime = rospy.Duration()

		vector_marker = Marker()
		vector_marker.header = msg.header
		vector_marker.ns = "vector"
		vector_marker.id = 2
		vector_marker.type = Marker.LINE_STRIP
		vector_marker.action = Marker.ADD
		
		vector_marker.scale.x = 0.05  # Line width
		
		vector_marker.color.r = 0.0
		vector_marker.color.g = 1.0
		vector_marker.color.b = 0.0
		vector_marker.color.a = 1.0

		vector_marker.points = [
			Point(0.0, 0.0, 0.0),
			msg.pose.position
		]

		vector_marker.lifetime = rospy.Duration()

		square_marker = Marker()
		square_marker.header = msg.header
		square_marker.ns = "uav_frame"
		square_marker.id = 3
		square_marker.type = Marker.LINE_LIST
		square_marker.action = Marker.ADD

		square_marker.scale.x = 0.05  # Line width

		square_marker.color.r = 1.0
		square_marker.color.g = 0.0
		square_marker.color.b = 0.0
		square_marker.color.a = 1.0

		# Define square points in UAV's local frame (425mm apart, converted to meters)
		half_size = 0.425 / 2
		square_points = [
			Point(half_size, half_size, 0.0),
			Point(-half_size, half_size, 0.0),
			Point(-half_size, -half_size, 0.0),
			Point(half_size, -half_size, 0.0),
			Point(half_size, half_size, 0.0)  # Close the square
		]

		# Transform square points to the UAV's pose
		transformed_points = []
		for point in square_points:
			transformed_point = self.transform_point(point, msg.pose)
			transformed_points.append(transformed_point)

		# Add lines connecting the square points (including diagonals)
		for i in range(len(transformed_points) - 1):
			square_marker.points.append(transformed_points[i])
			square_marker.points.append(transformed_points[i + 1])

		# Add diagonal lines
		square_marker.points.append(transformed_points[0])  # Top-right to bottom-left
		square_marker.points.append(transformed_points[2])

		square_marker.points.append(transformed_points[1])  # Top-left to bottom-right
		square_marker.points.append(transformed_points[3])

		square_marker.lifetime = rospy.Duration()

		# Publish all markers
		self.marker_pub.publish(uav_center_marker)
		self.marker_pub.publish(camera_marker)
		self.marker_pub.publish(vector_marker)
		self.marker_pub.publish(square_marker)

	def transform_point(self, point, pose):
		"""Transform a point from the UAV's local frame to the global frame."""
		# Convert pose orientation to a rotation matrix
		quaternion = [
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w
		]
		rotation_matrix = tf.quaternion_matrix(quaternion)

		# Apply rotation and translation
		local_point = [point.x, point.y, point.z, 1.0]
		global_point = rotation_matrix.dot(local_point)
		global_point[0] += pose.position.x
		global_point[1] += pose.position.y
		global_point[2] += pose.position.z

		return Point(global_point[0], global_point[1], global_point[2])

if __name__ == '__main__':
	try:
		visualizer = DistanceVisualizer()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass