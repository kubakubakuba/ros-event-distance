#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from event_distance_estimator.msg import LEDMarkers

class DummyLEDPublisher:
	def __init__(self):
		rospy.init_node("dummy_led_publisher")
		self.pub = rospy.Publisher("/detected_leds", LEDMarkers, queue_size=10)
		
		self.led_configs = [
			{
				"points": [[684, 111, 0], [537, 110, 0], [699, 179, 0], [527, 179, 0]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[734, 352, 0], [681, 424, 0], [525, 378, 0]],
				"ids": [0, 2, 3]
			}
		]
		
		self.rate = rospy.Rate(0.5)
		rospy.loginfo("Dummy LED Publisher ready")

	def run(self):
		while not rospy.is_shutdown():
			for config in self.led_configs:
				msg = LEDMarkers()
				msg.header = Header()
				msg.header.stamp = rospy.Time.now()
				msg.header.frame_id = "camera_frame"
				
				msg.points = []
				msg.ids = []
				
				for coords, led_id in zip(config["points"], config["ids"]):
					point = Point()
					point.x = coords[0]
					point.y = coords[1]
					point.z = coords[2]
					msg.points.append(point)
					msg.ids.append(led_id)
				
				self.pub.publish(msg)
				rospy.loginfo(f"Published LEDs: IDs {config['ids']}")
				self.rate.sleep()

if __name__ == '__main__':
	try:
		node = DummyLEDPublisher()
		node.run()
	except rospy.ROSInterruptException:
		pass