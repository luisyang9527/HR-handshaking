#!/usr/bin/env python3
__author__ = "Zhicheng Yang"
__copyright__ = "Copyright 2021, Technical University of Darmstadt"
__credits__ = ["Zhicheng Yang"]
__license__ = "GNU GPL v3.0"
__version__ = "1.0.6"
__maintainer__ = "Zhicheng Yang"
__email__ = "zhicheng.yang.luis@gmail.com"
__status__ = "Experimental"

import numpy as np
import rospy
from std_msgs.msg import String

class HandShakingController():
	def __init__(self):
		#实例化Subscriber对象
		rospy.Subscriber("festo/phand/state", HandState, self.hand_state_cb)
		rospy.Subscriber("festo/phand/connected_sensors/loomia_sensors", GenericSensor, self.loomia_msg_cb, queue_size=1)
		rospy.Subscriber("festo/phand/human_grip_force", Float32, queue_size=1))
		#实例化Publisher对象
		self.set_pressure_topic = rospy.Publisher("festo/phand/set_pressures", SimpleFluidPressures, queue_size=1)
		self.grip_pressure_topic = rospy.Publisher("festo/phand/human_grip_force", Float32, queue_size=1)
		self.robot_grip_pressure_topic = rospy.Publisher("festo/phand/robot_grip_force", Float32, queue_size=1)

	def loomia_msg_cb(self, msg):
		pressure_array = np.zeros(shape=[12,11])
		msg_length = len(msg.calibrated_values)

		for x in range(0,12):
			for y in range(0,11):
				index = (x * 11) + y 

				if index >= msg_length:
					break
				
				pressure_array[x,y] = msg.raw_values[index]	

		if not self.calibrated: # Using the first 20 values when no interaction is happenning as the offset
			self.zero_grip.append(pressure_array)
			if len(self.zero_grip) < 20:
				return
			else:
				self.zero_grip = np.array(self.zero_grip)
				self.loomia_offset = self.zero_grip.mean(0)
				self.calibrated = True

		self.pressure_array = pressure_array - self.loomia_offset
		# Obtained using trial and error. Need to update with loomia layout diagram from Timo
		grip_pressure = (self.pressure_array[6:9, 6:].sum() + self.pressure_array[9:, 6:8].sum())/21 
		# grip_pressure = (self.pressure_array[4:9, 0].sum() + self.pressure_array[0:9, 3:5].sum() + self.pressure_array[7:11, 6:11].sum()) / 43
		self.grip_pressure_topic.publish(Float32(grip_pressure))

		#robot_grip_pressure = (self.pressure_array[2:6, 3:4].sum() + self.pressure_array[0:6, 4:12].sum() + self.pressure_array[6:11, 0:3].sum()) / 67
		#robot_grip_pressure = (self.pressure_array[0, 6:11].sum() + self.pressure_array[9:12, 6:11].sum() + self.pressure_array[0:10, 0: 6].sum()) / 80
		#self.robot_grip_pressure_topic.publish(Float32(robot_grip_pressure))

		# Max and Min values observed using rqt_plot
		min_grip = 0.
		max_grip = 200.
		grip_pressure = (np.clip(grip_pressure, min_grip, max_grip) - min_grip)/(max_grip - min_grip)*100
		#self.sliders[0].setValue(100 * grip_pressure)

		#robot_grip_pressure = (np.clip(robot_grip_pressure, min_grip, max_grip) - min_grip) / (max_grip - min_grip)
		# self.sliders[0].setValue(100 * robot_grip_pressure)

	def controller(ctl):
		if ctl = 'P':
			last_contact= false 
			in_contact= check_contact()
			state.closure.clear()
			if in_contact:
				q = k_p * grip_presssure + q0
				outFile << Arr[0] << ", " << Arr[1]<< ", "<< Arr[2] << ", " << cb.closure << ", "<< cb.current <<endl;

			state.closure.push_back(q); //round the closure value to the closest integer

		}else{
			// not in contact
			if(last_contact!=in_contact)
			outFile << 0 << ", " << 0<< ", "<< 0 << ", " << 0 << ", "<< 0 <<endl;
			state.closure.push_back((int) 0 );
		}
		pub.publish(state);
		ros::spinOnce();
		//		usleep(1000);  //dynamic usleeps takes microseconds in input
		loop_rate.sleep();
		last_contact=in_contact;
	}
		


if __name__ == "__main__":
    	#2.初始化 ROS 节点:命名(唯一)
	rospy.init_node("P_control_node")
    	
    #4.组织被发布的数据，并编写逻辑发布数据
    msg = String()  #创建 msg 对象
    msg_front = "hello 你好"
    count = 0  #计数器 
    # 设置循环频率
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        #拼接字符串
        msg.data = msg_front + str(count)

        pub.publish(msg)
        rate.sleep()
        rospy.loginfo("写出的数据:%s",msg.data)
        count += 1

def my_ctl(ctl, q, qd, q_des, qd_des, qdd_des, q_hist, q_deshist, gravity, coriolis, M):
    Kp = np.array([60, 30])
    Kd = np.array([10, 6])
    Ki = np.array([0.1, 0.1])
    if ctl == 'P':
        u = np.zeros((2, 1))  # Implement your controller here       
        u = np.multiply(Kp, (q_des - q)).reshape(2, 1)
    elif ctl == 'PD':
        u = np.zeros((2, 1))  # Implement your controller here       
        u = (np.multiply(Kp, (q_des - q)) + np.multiply(Kd, (qd_des - qd))).reshape(2, 1)
    elif ctl == 'PID':
        u = np.zeros((2, 1))  # Implement your controller here       
        u = (np.multiply(Kp, (q_des - q)) + np.multiply(Kd, (qd_des - qd)) + np.multiply(Ki, np.sum((q_deshist - q_hist), 0))).reshape(2, 1)
    return u

/*
 * ctrl_1.cpp
 *
 *  Created on: Aug 31, 2018
 *      Author: Francesco Vigni
 *
 * ctrl_1.cpp implements a closed loop controller for human-robot
 * handshake. The q-force relationship follows a law like : F= k(q0-q)
 * a trigger is present on sensor id 3 in order to switch on and off the
 * handshaking
 *
 */

#include <std_msgs/Float32MultiArray.h>
#include <functions.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>

using namespace std;
string save_file(const string& name);

int main(int argc, char **argv)
{
	int q;
	string id_controller = "1";

	if (argc < 3) {
		// Tell the user how to run the program
		show_usage(argv[0]);
		/* "Usage messages" are a conventional way of telling the user
		 * how to run a program if they enter the command incorrectly.
		 */
		return 1;
	}
	int q0;float p[3];

	if(*argv[1]=='1'){
		q0=8500;
	}else if(*argv[1]=='2'){
		q0=9000;
	}else if(*argv[1]=='3'){
		q0=10000;
	}else if(*argv[1]=='4'){
		q0=11000;
	}else if(*argv[1]=='5'){
		q0=12000;

	}else {
		show_usage(argv[0]);
		cout << "handsize code out of bounds" << std::endl;
		return 1;
	}
	string id_participant = argv[2];

	cout <<"Participant ID = "<< id_participant <<"\ninitial position set to " << q0 <<endl;

	ros::init(argc, argv, "ctrl_" + id_controller);
	ros::NodeHandle n;
	ros::Rate loop_rate(100); //Hz
	ros::Subscriber sub = n.subscribe("sensors_FSR", 100, arrayCallback_sensors);
	ros::Publisher pub = n.advertise<qb_interface::handRef>("/qb_class/hand_ref", 100);
	qb_interface::handPos state;
	cout << "INFO -> Controller " << id_controller << " started"<<endl;

	ros::param::set("/stiffness",1.0);

	// Saving file routine
	callbacks cb;
	ros::Subscriber sub1 = n.subscribe("/qb_class/hand_ref",100, &callbacks::cb_closure, &cb);
	ros::Subscriber sub2 = n.subscribe("/qb_class/hand_measurement",100, &callbacks::cb_current, &cb);
	ros::Subscriber sub3 = n.subscribe("sensors_FSR", 100, arrayCallback_sensors);

	// Saving File routine
	string name = "c"+ id_controller + "_id" + id_participant + "_n";
	//save_file checks if a file exists in a specific folder
	//it true create a new file with a incremental number in the name


	std::ofstream outFile;
	cout << "saving file: " << save_file(name) << endl;
	outFile.open(save_file(name));



	bool last_contact= false ;
	while (ros::ok())
	{
		bool in_contact= check_contact();
		state.closure.clear();
		if(in_contact){

			//  in contact


			q=compute_f_with_q0(q0);
			outFile << Arr[0] << ", " << Arr[1]<< ", "<< Arr[2] << ", " << cb.closure << ", "<< cb.current <<endl;

			state.closure.push_back(q); //round the closure value to the closest integer

		}else{
			// not in contact
			if(last_contact!=in_contact)
			outFile << 0 << ", " << 0<< ", "<< 0 << ", " << 0 << ", "<< 0 <<endl;
			state.closure.push_back((int) 0 );
		}
		pub.publish(state);
		ros::spinOnce();
		//		usleep(1000);  //dynamic usleeps takes microseconds in input
		loop_rate.sleep();
		last_contact=in_contact;
	}
	outFile.close();

}