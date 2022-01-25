#!/usr/bin/env python3

__author__ = "Vignesh Prasad"
__copyright__ = "Copyright 2021, Technical University of Darmstadt"
__credits__ = ["Vignesh Prasad"]
__license__ = "GNU GPL v3.0"
__version__ = "1.0.6"
__maintainer__ = "Vignesh Prasad"
__email__ = "vignesh.prasad@tu-darmstadt.de"
__status__ = "Experimental"

import signal, sys
import numpy as np
from scipy.ndimage.filters import uniform_filter1d

import roslib; roslib.load_manifest('haptic_emotions')
import rospy
from urdf_parser_py.urdf import URDF

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from festo_phand_msgs.srv import *
from festo_phand_msgs.msg import *

import logging

import matplotlib.pyplot as plt

class HandController:

	def __init__(self):

		self.joint_pressures = []
		
		self.joints = ["Thumb deviation",
					   "Thumb lower",
					   "Pressure spring",
					   "Thumb upper",
					   "Index upper",
					   "Cylinder left",
					   "Index lower",
					   "Cylinder right",
					   "Middle&ring low",
					   "Index deviation",
					   "Middle&ring top",
					   "pinky",
					   ]
		
		self.low_joints = [ "Thumb lower",
							"Thumb upper",
							"Index upper",
							"Index lower",
							"Middle&ring low",
							"Middle&ring top",
							"pinky"
					   ]

		self.low_joints_idx = [self.joints.index(j) for j in self.low_joints] 

		self.joint_limits = [5.0] * len(self.joints)
		self.loomia_array = np.zeros(shape=[12,11])

		rospy.Subscriber("festo/phand/connected_sensors/loomia_sensors", GenericSensor, self.loomia_msg_cb, queue_size=1)

		self.set_pressure_topic = rospy.Publisher("festo/phand/set_pressures", SimpleFluidPressures, queue_size=1)
		self.command_topic = rospy.Publisher("festo/phand/control_command", Float32, queue_size=1)
		self.filter_command_topic = rospy.Publisher("festo/phand/control_command_filtered", Float32, queue_size=1)
		self.force_human_topic = rospy.Publisher("festo/phand/human_grip_force", Float32, queue_size=1)

		self.reset()

		self.calibrated = False
		self.recalibration = False
		self.zero_grip = []
		self.zero_grip_recalib = []

		self.control_history = np.zeros(25)
        
	def loomia_msg_cb(self, msg):
		loomia_array = np.zeros(shape=[12,11])
		msg_length = len(msg.calibrated_values)

		for x in range(0,12):
			for y in range(0,11):
				index = (x * 11) + y 

				if index >= msg_length:
					break
				
				loomia_array[x,y] = msg.raw_values[index]
				if loomia_array[x,y] > 1000:
					loomia_array[x, y] = 0
		# self.loomia_offset = 0
		if not self.calibrated: # Using the first 20 values when no interaction is happenning as the offset
			self.zero_grip.append(loomia_array)
			if len(self.zero_grip) < 20:
				return
			else:
				self.zero_grip = np.array(self.zero_grip)
				self.loomia_offset = self.zero_grip.mean(0)
				self.calibrated = True
		self.loomia_array = loomia_array - self.loomia_offset
		# force_human = self.loomia_array[0:9, 6:].sum()/(self.loomia_array[0:9, 6:].shape[0]*self.loomia_array[0:9, 6:].shape[1]) # Obtained using trial and error. Need to update with loomia layout diagram from Timo
		force_human = self.loomia_array[6:9, 6:].sum()/(self.loomia_array[6:9, 6:].shape[0]*self.loomia_array[6:9, 6:].shape[1])
		
		if force_human < 0 and not self.recalibration:
			self.recalibration = True
			self.zero_grip_recalib = []

		if self.recalibration:
			self.zero_grip_recalib.append(loomia_array)
			if len(self.zero_grip_recalib) == 20:
				self.zero_grip = np.array(self.zero_grip_recalib)
				self.loomia_offset = self.zero_grip.mean(0)
				self.recalibration = False
				self.loomia_array = loomia_array - self.loomia_offset
				# force_human = self.loomia_array[0:9, 6:].sum()/(self.loomia_array[0:9, 6:].shape[0]*self.loomia_array[0:9, 6:].shape[1]) # Obtained using trial and error. Need to update with loomia layout diagram from Timo
				force_human = self.loomia_array[6:9, 6:].sum()/(self.loomia_array[6:9, 6:].shape[0]*self.loomia_array[6:9, 6:].shape[1])

		# Max and Min values observed using rqt_plot
		min_grip = min(0., force_human)
		max_grip = max(200., force_human)
		force_human = 25 * (np.clip(force_human, min_grip, max_grip) - min_grip)/(max_grip - min_grip)
		self.force_human_topic.publish(Float32(force_human))

		self.control_history[:-1] = self.control_history[1:]
		self.control_history[-1] = force_human
		filtered_force_human = np.mean(uniform_filter1d(self.control_history, size=5))

		self.joint_pressures = []
		for joint in self.joints:
			if joint in self.low_joints:
				self.joint_pressures.append((filtered_force_human+1)*1e5)
			# elif joint=='Cylinder left'or joint=='Pressure spring':
			elif joint=='Cylinder left':
				self.joint_pressures.append(3e5)
			else:
				self.joint_pressures.append(0.0)
		
		self.joint_pressures[0] = (self.joint_limits[0] - filtered_force_human+1)*1e5


		# msg = SimpleFluidPressures()
		# msg.values = self.joint_pressures
		# self.set_pressure_topic.publish(msg)
		self.command_topic.publish(Float32(self.control_history[-1]))
		self.filter_command_topic.publish(Float32(filtered_force_human))

	def reset(self):
		self.joint_pressures = []
		for joint in self.joints:
			if joint in self.low_joints:
				self.joint_pressures.append(0)
			elif joint=='Cylinder left':
				self.joint_pressures.append(3e5)
			else:
				self.joint_pressures.append(0.0)

		msg = SimpleFluidPressures()
		msg.values = self.joint_pressures
		self.set_pressure_topic.publish(msg)

if __name__ == '__main__':
	rospy.init_node('basic_interactive_handshake_node')
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	controller = HandController()

	# Start the event loop.
	rospy.spin()

	# controller.reset()