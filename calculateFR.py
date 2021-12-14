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

import roslib; roslib.load_manifest('haptic_emotions')
import rospy
from urdf_parser_py.urdf import URDF

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

from festo_phand_msgs.srv import *
from festo_phand_msgs.msg import *

import logging

from controller import*

class HandPressureSlider(QSlider):
	valueUpdated = pyqtSignal(float)
	def __init__(self, joint_name, max_pressure):
		super(HandPressureSlider, self).__init__(Qt.Horizontal)
		self.joint_name = joint_name
		self.max_pressure = max_pressure

		self.setMinimumWidth(300)
		self.setMinimum(0)
		self.setMaximum(100)
		self.setTickPosition(0)
		self.setTickPosition(QSlider.TicksBelow)

		self.valueChanged[int].connect(self.map_value)

		self.setStyleSheet("QSlider::groove:horizontal { border: 1px solid #999999; height: 8px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4); margin: 2px 0;}"
						   "QSlider::handle:horizontal { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f); "
						   "border: 1px solid #5c5c5c;height:30px;width: 30px;margin: -5px 0; border-radius: 3px;}")

	@property
	def joint_value(self):
		return self.value() * self.max_pressure/100

	def map_value(self, value):
		self.valueUpdated.emit(self.joint_value)

class HandControllerGui(QWidget):

	handstate_update = pyqtSignal()

	def __init__(self):
		super(QWidget, self).__init__()
		self.last_time = 0
		self.send_threshold = 0.01
		self.hand_state = HandState()
		self.sliders = []
		self.pressure_lbls = []
		self.joint_pressures = []
		
		self.setStyleSheet("font-family: 'Arial';")

		self.gridLayout = QGridLayout(self)

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

		self.joint_limits = [5.0] * len(self.joints)
		self.pressure_array = np.zeros(shape=[12,11])

		self.setup_gui()

		rospy.Subscriber("festo/phand/state", HandState, self.hand_state_cb)
		rospy.Subscriber("festo/phand/connected_sensors/loomia_sensors", GenericSensor, self.loomia_msg_cb, queue_size=1)

		self.set_pressure_topic = rospy.Publisher("festo/phand/set_pressures", SimpleFluidPressures, queue_size=1)
		self.grip_pressure_topic = rospy.Publisher("festo/phand/human_grip_force", Float32, queue_size=1)

		#For robot_force
		self.robot_pressure_topic = rospy.Publisher("festo/phand/robot_force", Float32, queue_size=1)
	
		self.calibrated = False
		self.zero_grip = []

		self.show()


	def update_pressure_lables(self):
		for num, pressure in enumerate(self.hand_state.internal_sensors.actual_pressures.values):
			# self.pressure_lbls[num].setNum( round(pressure/1e5-1,2))
			self.pressure_lbls[num].setText("%.2f" % round(pressure/1e5-1, 2) )

	def loomia_msg_cb(self, msg):
		pressure_array = np.zeros(shape=[12,11])
		msg_length = len(msg.calibrated_values)

		for x in range(0,12):
			for y in range(0,11):
				index = (x * 11) + y 

				if index >= msg_length:
					break
				
				pressure_array[x,y] = msg.raw_values[index]	
				if pressure_array[x,y] > 1000:
					pressure_array[x, y] = 0
		#define the offset
		# self.loomia_offset = 0
		if not self.calibrated: # Using the first 20 values when no interaction is happenning as the offset
			self.zero_grip.append(pressure_array)
			if len(self.zero_grip) < 20:
				return
			else:
				self.zero_grip = np.array(self.zero_grip)
				self.loomia_offset = self.zero_grip.mean(0)
				self.calibrated = True
		self.pressure_array = pressure_array - self.loomia_offset
		# loomia mapping
		grip_pressure = self.pressure_array[6:9, 6:].sum()/(self.pressure_array[6:9, 6:].shape[0]*self.pressure_array[6:9, 6:].shape[1]) 
		# trial
		# grip_pressure = self.pressure_array[6:11, 6:].sum()/(self.pressure_array[6:11, 6:].shape[0]*self.pressure_array[6:11, 6:].shape[1]) 
		self.grip_pressure_topic.publish(Float32(grip_pressure))

		#TODO: calculate and publish FR
		# loomia mapping
		robot_pressure = (self.pressure_array[0:9, 0:6].sum() + self.pressure_array[9:, 6:].sum())/(self.pressure_array[0:9, 0:6].shape[0]*self.pressure_array[0:9, 0:6].shape[1] + self.pressure_array[9:, 6:].shape[0]*self.pressure_array[9:, 6:].shape[1])
		# trial
		# robot_pressure = (self.pressure_array[0:9, 0:6].sum() + self.pressure_array[9:, 6:].sum() + self.pressure_array[0, 6:].sum())/(self.pressure_array[0:9, 0:6].shape[0]*self.pressure_array[0:9, 0:6].shape[1] + self.pressure_array[9:, 6:].shape[0]*self.pressure_array[9:, 6:].shape[1] + 5)
		# using pid controll 
		pid_controller = PID(1.029, 0.00, 0.00000000)
		pid_controller.reset()
		pid_controller.set_sample_time(0.01)
		self.actual = robot_pressure
		print("self.actual = {}".format(self.actual))
		# self.desired = 100 # reference force, you can set it by yourself
		self.desired = grip_pressure
		print("self.desired = {}".format(self.desired))
		# actual_force = pid_controller.update(self.actual, self.desired) + self.actual
		# print("actual_force = {}".format(actual_force))
		# self.robot_pressure_topic.publish(Float32(actual_force))

		for i in range(2):
			self.actual = pid_controller.update(self.actual, self.desired) + self.actual
			time.sleep(0.01)
		print("actual_force = {}".format(self.actual))
		self.robot_pressure_topic.publish(Float32(self.actual))

		# Max and Min values observed using rqt_plot
		min_grip = 0.
		max_grip = 200.
		# grip_pressure = (np.clip(grip_pressure, min_grip, max_grip) - min_grip)/(max_grip - min_grip)
		set_grip_pressure = (np.clip(self.actual, min_grip, max_grip) - min_grip)/(max_grip - min_grip)
		self.sliders[0].setValue(100 * set_grip_pressure)
		# print("grip_pressure = {}".format(grip_pressure))

	def hand_state_cb(self, msg):
		self.hand_state = msg
		self.handstate_update.emit()
	
	def setup_gui(self):
		row = 0
		joint = 'Open -- Close'
		lbl = QLabel(str(joint))
		lbl_value = QLabel()
		lbl_value.setNum(0.00)

		lbl_value.setMinimumWidth(50)
		lbl_value.setMaximumWidth(50)

		self.sliders.append(HandPressureSlider(str(joint), self.joint_limits[row]))
		self.sliders[row].valueUpdated.connect(lbl_value.setNum)
		self.sliders[row].valueUpdated.connect(self.btn_click)

		self.gridLayout.addWidget(lbl, row, 0)
		self.gridLayout.addWidget(self.sliders[row], row, 1)
		self.gridLayout.addWidget(lbl_value, row, 2)

		lbl = QLabel(str(0.0))
		self.pressure_lbls.append(lbl)
		self.gridLayout.addWidget(lbl, 1, 4)

		lbl = QLabel(str(0.0))
		self.pressure_lbls.append(lbl)
		self.gridLayout.addWidget(lbl, 1, 2)
	
	def generate_publish_data(self):
		self.joint_pressures = []
		for joint in self.joints:
			if joint in self.low_joints:
				self.joint_pressures.append((self.sliders[0].joint_value+1)*1e5)
			# elif joint=='Cylinder left'or joint=='Pressure spring':
			elif joint=='Cylinder left':
				self.joint_pressures.append(3e5)
			else:
				self.joint_pressures.append(0.0)
		
		self.joint_pressures[0] = (self.sliders[0].max_pressure - self.sliders[0].joint_value+1)*1e5

	def btn_click(self):

		if rospy.get_time() - self.last_time > self.send_threshold:
			self.last_time = rospy.get_time()
			self.generate_publish_data()
			msg = SimpleFluidPressures()
			msg.values = self.joint_pressures
			self.set_pressure_topic.publish(msg)


if __name__ == '__main__':
	rospy.init_node('FH_node')
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	app = QApplication(sys.argv)
	ui = HandControllerGui()

	# Start the event loop.
	sys.exit(app.exec_())