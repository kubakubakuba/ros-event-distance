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
				"points": [[734, 352, 10], [681, 424, 10], [525, 378, 10]],
				"ids": [0, 2, 3]
			},
			{
				"points": [[684, 111, 10], [537, 110, 10], [699, 179, 10], [527, 179, 10]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[538, 393, 10], [713, 407, 10], [563, 321, 10], [706, 330, 10]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[669, 358, 10], [569, 354, 10], [672, 393, 10], [560, 387, 10]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[694, 373, 10], [615, 399, 10], [542, 370, 10]],
				"ids": [0, 2, 3]
			},
			{
				"points": [[672, 390, 10], [663, 356, 10], [559, 388, 10], [563, 354, 10]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[616, 396, 10], [689, 371, 10], [541, 367, 10]],
				"ids": [0, 1, 2]
			},
			{
				"points": [[694, 353, 7], [646, 367, 7], [598, 359, 7]],
				"ids": [0, 2, 3]
			},
			{
				"points": [[685, 363, 7], [681, 347, 7], [614, 366, 7], [614, 351, 7]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[650, 366, 7], [698, 352, 7], [601, 359, 7]],
				"ids": [0, 1, 2]
			},
			{
				"points": [[621, 366, 7], [692, 360, 7], [613, 352, 7], [679, 346, 7]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[652, 383, 5], [658, 377, 5], [611, 385, 5], [617, 379, 5]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[91, 466, 5], [93, 459, 5], [67, 472, 5], [69, 465, 5]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[1090, 376, 5], [1095, 368, 5], [1055, 372, 5], [1061, 366, 5]],
				"ids": [0, 1, 2, 3]
			},
			{
				"points": [[664, 47, 5], [671, 41, 5], [621, 47, 5], [627, 42, 5]],
				"ids": [0, 1, 2, 3]
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
				rospy.loginfo(f"Published LEDs with IDs {config['ids']} at positions {config['points']}")
				self.rate.sleep()

if __name__ == '__main__':
	try:
		node = DummyLEDPublisher()
		node.run()
	except rospy.ROSInterruptException:
		pass