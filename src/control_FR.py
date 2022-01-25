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

# from test_pid import*
from PID_controller import*
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
		return self.value()

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
		self.filter_force_topic = rospy.Publisher("festo/phand/control_force_filtered", Float32, queue_size=1)
		self.command_topic = rospy.Publisher("festo/phand/control_command", Float32, queue_size=1)
		
		self.calibrated = False
		# self.recalibration = False

		self.zero_grip_human = []
		self.zero_grip_robot = []
		# self.zero_grip_recalib = []
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
		human_pressure = self.pressure_array[6:9, 6:].sum() / 15
		# robot_pressure = (self.pressure_array[0:8, 0].sum() +
		# 					self.pressure_array[0:8, 3:6].sum() +
		# 					self.pressure_array[9:, 6:8].sum() +
		# 					self.pressure_array[9:, 9:].sum()) / (7+21+6+6)
		robot_pressure = (self.pressure_array[:4,:3].sum() + self.pressure_array[:4, 4:6].sum() +
							self.pressure_array[4:8, :3].sum() + self.pressure_array[4:8, 5].sum() +
							self.pressure_array[8, 2:6].sum() +
							self.pressure_array[9:11, 6].sum() + self.pressure_array[9:11, 8].sum() + self.pressure_array[11, 10]) / 41
		if not self.calibrated: # Using the first 20 values when no interaction is happenning as the offset
			self.zero_grip_human.append(human_pressure)
			self.zero_grip_robot.append(robot_pressure)
			if len(self.zero_grip_human) < 20 and len(self.zero_grip_robot) < 20:
				return
			else:
				self.zero_grip_human = np.array(self.zero_grip_human)
				self.zero_grip_robot = np.array(self.zero_grip_robot)
				self.loomia_offset_human = self.zero_grip_human.mean()
				self.loomia_offset_robot = self.zero_grip_robot.mean()
				self.calibrated = True
		human_pressure -= self.loomia_offset_human
		robot_pressure -= self.loomia_offset_robot
		
		# if robot_pressure < 0 and not self.recalibration:
		# 	self.recalibration = True
		# 	self.zero_grip_recalib = []
		# if self.recalibration:
		# 	self.zero_grip_recalib.append(self.pressure_array)
		# 	if len(self.zero_grip_recalib) == 20:
		# 		self.zero_grip = np.array(self.zero_grip_recalib)
		# 		self.loomia_offset = self.zero_grip.mean(0)
		# 		self.recalibration = False
		# 		self.pressure_array = self.pressure_array - self.loomia_offset
		# 		robot_pressure = (self.pressure_array[0:8, 0].sum() + self.pressure_array[0:8, 3:6].sum() + self.pressure_array[9:, 6:8].sum() + self.pressure_array[9:, 9:].sum()) / (7+21+6+6)

		self.actual = robot_pressure
		desired = human_pressure
		# print("self.desired = {}".format(self.desired))

		pid = PID(1.02, 0.76, 0.12)
		pid.reset()
		for i in range(5):
			pid.update(self.actual, desired)
			time.sleep(0.01)

		robot_force = self.actual + pid.output
		self.control_history[:-1] = self.control_history[1:]
		self.control_history[-1] = robot_force
		filtered_force_robot = np.mean(uniform_filter1d(self.control_history, size=5))

		# Max and Min values observed using rqt_plot
		min_grip = 0
		max_grip = max(180, filtered_force_robot)
		filtered_command = 5 * (np.clip(filtered_force_robot, min_grip, max_grip) - min_grip) / (max_grip - min_grip)

		self.grip_pressure_topic.publish(Float32(desired))
		self.robot_pressure_topic.publish(Float32(robot_force))
		# self.sliders[0].setValue(actual_force)


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
				self.joint_pressures.append(2e5)
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