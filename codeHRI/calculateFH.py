#!/usr/bin/env python3

__author__ = "Zhicheng Yang"
__copyright__ = "Copyright 2021, Technical University of Darmstadt"
__credits__ = ["Zhicheng Yang"]
__license__ = "GNU GPL v3.0"
__version__ = "1.0.6"
__maintainer__ = "Zhicheng Yang"
__email__ = "zhicheng.yang@stud.tu-darmstadt.de"
__status__ = "Experimental"

import matplotlib

from bionic_serial_client.bionic_serial_client import BionicSerialClient
from bionic_message_tools.bionic_message_tools import BionicMessageHandler
from phand_messages.loomia_messages import BionicLoomiaMessage, BionicSetLoomiaValuesActionMessage
from phand_messages.phand_message_constants import BIONIC_MSG_IDS

import copy
import logging

import rospy
from festo_phand_msgs.srv import LoomiaSensorConfig, LoomiaSensorConfigRequest
from festo_phand_msgs.msg import GenericSensor

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import Animation
from matplotlib.widgets import Button
import time

class LoomiaTestFH:

    def __init__(self):

        logging.basicConfig(level=logging.INFO)
       
        rospy.init_node("loomia_test_FH")
        
        rospy.wait_for_service("festo/phand/loomia/set_configuration")
        self.set_loomia_settings = rospy.ServiceProxy("festo/phand/loomia/set_configuration", LoomiaSensorConfig)

        rospy.Subscriber("festo/phand/connected_sensors/loomia_sensors", GenericSensor, callback=self.loomia_msg_cb, queue_size=1)
        self.grip_pressure_topic = rospy.Publisher("festo/phand/festo_grip_force", Float32, queue_size=1)

        self._shutdown = False
        self.adc_reference_voltage = 2.2
        self.series_resistance_sensors = [1000]*11
        self.d_column_switch = 75
        self.logo_led = 0
        self.onboard_led = 0
        
        msg_handler = BionicMessageHandler()
        msg = BionicLoomiaMessage(BIONIC_MSG_IDS.LOOMIA_MSG_ID)
        msg.register_cb(self.loomia_msg_cb)
        msg_handler.register_message_type(msg)

        self.calibrated = False
        self.zero_grip = []
        self.grip_pressure = 0

        self.pressure_array = np.zeros(shape=[12,11])     
        self.init_plot()   

        self.loomia_config_msg = LoomiaSensorConfigRequest()
        self.loomia_config_msg.series_resistance = self.series_resistance_sensors
        self.loomia_config_msg.reference_voltage = self.adc_reference_voltage
        self.loomia_config_msg.d_column_switch = self.d_column_switch
        self.loomia_config_msg.led_logo = self.logo_led
        self.loomia_config_msg.led_board = self.onboard_led
        
        while not self._shutdown:
            self.update_plot()            
        
    def init_plot(self):

        self.fig, self.ax = plt.subplots()  
        ledboard = plt.axes([0.81, 0.05, 0.1, 0.075])
        plt.subplots_adjust(bottom=0.2)          
        bLedBoard = Button(ledboard, 'BOARD LED')
        bLedBoard.on_clicked(self.button_led_board_cb) 
        self.ax.imshow(self.pressure_array)                   

    def update_plot(self):

        self.ax.cla() 

        for x in range(0,12):
            for y in range(0,11):
                self.ax.text(y, x, self.pressure_array[x, y], ha="center", va="center", color="w")  

        ledboard = plt.axes([0.81, 0.05, 0.1, 0.075])
        plt.subplots_adjust(bottom=0.2)          
        bLedBoard = Button(ledboard, 'BOARD LED')
        bLedBoard.on_clicked(self.button_led_board_cb) 

        self.ax.imshow(self.pressure_array)
        plt.pause(0.05)        

    def button_led_board_cb(self, event):
        
        self.logo_led = not self.logo_led
        self.onboard_led = not self. onboard_led
        print (f"MESSAGE SENT: {self.set_loomia_cfg()}")

    def loomia_msg_cb(self, msg):
        
        msg_length = len(msg.calibrated_values)

        for x in range(0,12):
            for y in range(0,11):
                index = (x * 11) + y 

                if index >= msg_length:
                    break
                
                self.pressure_array[x,y] = msg.raw_values[index]     
        if not self.calibrated: # Using the first 20 values when no interaction is happenning as the offset
			self.zero_grip.append(self.pressure_array)
			if len(self.zero_grip) < 20:
				return
			else:
				self.zero_grip = np.array(self.zero_grip)
				self.loomia_offset = self.zero_grip.mean(0)
				self.calibrated = True

		self.pressure_array = self.pressure_array - self.loomia_offset
        # index_array = np.argwhere(self.pressure_array > 5)
        # print(index_array)
    
		self.grip_pressure = self.pressure_array[self.pressure_array > 0].sum() / self.pressure_array[self.pressure_array > 0].shape
		self.grip_pressure_topic.publish(Float32(grip_pressure))

		# Max and Min values observed using rqt_plot
		min_grip = 0.
		max_grip = 200.
		self.grip_pressure = (np.clip(grip_pressure, min_grip, max_grip) - min_grip)/(max_grip - min_grip)

    def set_loomia_cfg(self):

        self.loomia_config_msg.led_logo = self.logo_led
        self.loomia_config_msg.led_board = self.onboard_led

        return self.set_loomia_settings(self.loomia_config_msg)

if __name__ == "__main__":
    LoomiaTestFH()
