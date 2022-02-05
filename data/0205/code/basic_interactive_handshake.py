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
from calibrition import *

import logging

import matplotlib.pyplot as plt
from scipy.ndimage.filters import uniform_filter1d

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
		# self.joint_pressures = []
		
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
		self.robot_pressure_topic = rospy.Publisher("festo/phand/robot_force", Float32, queue_size=1)
		self.filter_force_topic = rospy.Publisher("festo/phand/control_force_filtered", Float32, queue_size=1)
		self.command_topic = rospy.Publisher("festo/phand/control_command", Float32, queue_size=1)
		
		self.calibrated = False
		self.loomia_offset = 0
		self.zero_grip = []
		self.control_history = np.zeros(25)

		self.show()


	def update_pressure_lables(self):
		for num, pressure in enumerate(self.hand_state.internal_sensors.actual_pressures.values):
			self.pressure_lbls[num].setText("%.2f" % round(pressure/1e5-1, 2))

	def loomia_msg_cb(self, msg):
		msg_length = len(msg.calibrated_values)
		for x in range(0,12):
			for y in range(0,11):
				index = (x * 11) + y
				if index >= msg_length:
					break
				self.pressure_array[x,y] = msg.raw_values[index]

		if not self.calibrated:
			self.zero_grip.append(self.pressure_array)
			if len(self.zero_grip) == 100:
				self.zero_grip = np.array(self.zero_grip)
				self.loomia_offset = self.zero_grip.mean(0)
				self.calibrated = True
		self.pressure_array -= self.loomia_offset

		grip_pressure = np.clip(self.pressure_array[7:9, 6:], 0, 1000).sum() / 10
		robot_grip_pressure = (np.clip(self.pressure_array[:9, :2], 0, 1000).sum()
							+ np.clip(self.pressure_array[:9,3:6], 0, 1000).sum()) / 70

		self.control_history[:-1] = self.control_history[1:]
		self.control_history[-1] = grip_pressure
		filtered_force_human = np.mean(uniform_filter1d(self.control_history, size=5))

		# Max and Min values observed using rqt_plot
		min_grip = 0
		max_grip = max(200, grip_pressure)
		filtered_command = (np.clip(grip_pressure, min_grip, max_grip) - min_grip) / (max_grip - min_grip)

		self.grip_pressure_topic.publish(Float32(grip_pressure))
		self.robot_pressure_topic.publish(Float32(robot_grip_pressure))
		self.filter_force_topic.publish(Float32(filtered_force_human))
		self.command_topic.publish(Float32(filtered_command))
		

		if filtered_command > 0.2:
			self.sliders[0].setValue(filtered_command * 100)
		else:
			self.sliders[0].setValue(filtered_command * 0)

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
			elif joint=='Cylinder left':
				self.joint_pressures.append(3 * 1e5)
			elif joint=='Pressure spring':
				self.joint_pressures.append(3 * 1e5)
			elif joint == 'Index deviation':
				self.joint_pressures.append(3 * 1e5)
			else:
				self.joint_pressures.append(1e5)
		
		# self.joint_pressures[0] = (self.sliders[0].max_pressure - self.sliders[0].joint_value+1)*1e5

	def btn_click(self):

		if rospy.get_time() - self.last_time > self.send_threshold:
			self.last_time = rospy.get_time()
			self.generate_publish_data()
			msg = SimpleFluidPressures()
			msg.values = self.joint_pressures
			self.set_pressure_topic.publish(msg)


if __name__ == '__main__':
	rospy.init_node('basic_interactive_handshake_node')
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	app = QApplication(sys.argv)
	ui = HandControllerGui()

	# Start the event loop.
	sys.exit(app.exec_())