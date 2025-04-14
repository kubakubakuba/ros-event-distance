#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from matplotlib.animation import FuncAnimation

class DistanceTracker:
	def __init__(self):
		rospy.init_node('distance_tracker')
		
		self.pose_sub = rospy.Subscriber("/estimated_pose", PoseStamped, self.pose_callback)
		self.dist_pub = rospy.Publisher("/estimated_distance", Float32, queue_size=10)
		
		self.time_data = []
		self.distance_data = []
		self.start_time = rospy.get_time()

		plt.style.use('ggplot')
		self.fig, self.ax = plt.subplots(figsize=(10, 6))
		self.line, = self.ax.plot([], [], 'b-', linewidth=2)
		self.ax.set_title('UAV Distance from Camera Over Time')
		self.ax.set_xlabel('Time (s)')
		self.ax.set_ylabel('Distance (m)')
		self.ax.grid(True)

		self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
		plt.show(block=False)
		rospy.loginfo("Distance Tracker initialized")

	def pose_callback(self, msg):
		x = msg.pose.position.x
		y = msg.pose.position.y
		z = msg.pose.position.z
		distance = np.sqrt(x**2 + y**2 + z**2)

		current_time = rospy.get_time() - self.start_time
		self.time_data.append(current_time)
		self.distance_data.append(distance)
		
		dist_msg = Float32()
		dist_msg.data = distance
		self.dist_pub.publish(dist_msg)

		max_points = 100
		if len(self.time_data) > max_points:
			self.time_data = self.time_data[-max_points:]
			self.distance_data = self.distance_data[-max_points:]

	def update_plot(self, frame):
		if len(self.time_data) > 0:
			self.line.set_data(self.time_data, self.distance_data)
			self.ax.relim()
			self.ax.autoscale_view()
			
			if len(self.time_data) > 1:
				time_range = self.time_data[-1] - self.time_data[0]
				if time_range > 0:
					self.ax.set_xlim(self.time_data[0], self.time_data[-1])
		
		return self.line,

	def run(self):
		rospy.spin()
		self.fig.savefig('distance_plot.png')
		rospy.loginfo("Saved distance plot to distance_plot.png")

if __name__ == '__main__':
	try:
		tracker = DistanceTracker()
		tracker.run()
	except rospy.ROSInterruptException:
		pass