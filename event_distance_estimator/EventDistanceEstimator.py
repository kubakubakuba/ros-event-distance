#!/usr/bin/env python

import rospy
from event_distance_estimator.msg import ADCPacketRaw

import numpy as np
import time

class EventDistanceEstimator():
	def __init__(self):
		self.node = rospy.init_node("event_distance_estimator")

		# change this
		self.com_port = rospy.get_param("~com_port")
		self.com_baud = int(rospy.get_param("~com_baud"))
		self.com_cts = rospy.get_param("~com_cts")
		self.pga_gain = rospy.get_param("~pga_gain")
		self.pga_offset = rospy.get_param("~pga_offset")
		self.adc_channel = rospy.get_param("~adc_channel")
		self.adc_freq = rospy.get_param("~adc_freq")
		self.adc_samples_per_packet = rospy.get_param("~adc_samples_per_packet")
		self.adc_delim_seq = rospy.get_param("~adc_delim_seq")
		self.gain_adjustment_period = rospy.get_param("~gain_adjustment_period")
		self.settings_print_period = rospy.get_param("~settings_print_period")

		self.total_packet_bytes = len(self.adc_delim_seq) + 7 + int((2 * 3 * self.adc_samples_per_packet) // 4) + 2

		self.pub = rospy.Publisher("/event_distance_estimator_data", ADCPacketRaw, queue_size=1)

	def run(self):
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			# process the info
			pass

if __name__ == '__main__':
	estimator = EventDistanceEstimator()
	estimator.run()
	rospy.spin()