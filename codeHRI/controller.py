#!/usr/bin/env python3
__author__ = "Zhicheng Yang"
__copyright__ = "Copyright 2021, Technical University of Darmstadt"
__credits__ = ["Zhicheng Yang"]
__license__ = "GNU GPL v3.0"
__version__ = "1.0.6"
__maintainer__ = "Zhicheng Yang"
__email__ = "zhicheng.yang.luis@gmail.com"
__status__ = "Experimental"

import time

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

class PID:
    
    def __init__(self, p=0.2, i=0.0, d=0.0):

        self.reset()

        self.kp = p
        self.ki = i
        self.kd = d

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = time.time()

        #implenment the function
        self.reset()
        self.
        self.update(actual, desired)

    def reset(self):
        """
        Reset the PID parameters
        """

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 0.5

        self.output = 0.0

    def update(self, actual, desired):
        """
        Updates the PID value for the given reference feedback        
        """
        
        error = desired - actual

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        
        if (delta_time >= self.sample_time):
            self.p_term = self.kp * error
            self.i_term += error * delta_time
            
            if (self.i_term < -self.windup_guard):
                self.i_term = -self.windup_guard
            elif (self.i_term > self.windup_guard):
                self.i_term = self.windup_guard

            self.d_term = 0.0
            if delta_time > 0:
                self.d_term = delta_error / delta_time
            
            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            return self.p_term + (self.ki * self.i_term) + (self.kd * self.d_term)

    def set_kp(self, proportional_gain):
        """
        Determines how aggressively the PID reacts to the current error with setting Proportional Gain
        """

        self.kp = proportional_gain

    def set_ki(self, integral_gain):
        """
        Determines how aggressively the PID reacts to the current error with setting Integral Gain
        """

        self.ki = integral_gain

    def setKd(self, derivative_gain):
        """
        Determines how aggressively the PID reacts to the current error with setting Derivative Gain
        """

        self.kd = derivative_gain

    def set_windup(self, windup):
        """
        Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """

        self.windup_guard = windup

    def set_sample_time(self, sample_time):
        """
        PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """

        self.sample_time = sample_time



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

		# rospy.Subscriber("festo/phand/state", HandState, self.hand_state_cb)
        rospy.Subscriber("festo/phand/robot_force", Float32, self.pid_controll_cb, queue_size=1)

		self.set_pressure_topic = rospy.Publisher("festo/phand/set_pressures", SimpleFluidPressures, queue_size=1)
	
		self.calibrated = False
		self.zero_grip = []

		self.show()


	def update_pressure_lables(self):
		for num, pressure in enumerate(self.hand_state.internal_sensors.actual_pressures.values):
			# self.pressure_lbls[num].setNum( round(pressure/1e5-1,2))
			self.pressure_lbls[num].setText("%.2f" % round(pressure/1e5-1, 2) )

	def pid_controll_cb(self, msg):
        pid_controller = PID(p, i, d)
        pid_controller.reset()

        self.actual = msg.data
        self.desired = 10# reference force, you can set it by yourself
        actual = pid_controller.update(self.actual, self.desired)#update actual robot force until it approximataly equals to the deseired forece

        self.robot_actualForce_topic.publish(Float32(actual))

		# Max and Min values observed using rqt_plot
		min_grip = 0.
		max_grip = 200.
		actual_pressure = (np.clip(actual, min_grip, max_grip) - min_grip)/(max_grip - min_grip)
		self.sliders[0].setValue(100 * actual_pressure)#append the force to the sliders

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
			# print("last_time = {}".format(self.last_time))
			self.last_time = rospy.get_time()
			self.generate_publish_data()
			#if sum(self.joint_pressures) > 3e5:
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

if __name__ == "__main__":
	pid = PID(0.5, 0.2, 0.3)
	pid.reset()
	actual = robot_grip_force
	pid.update(actual, 3)
