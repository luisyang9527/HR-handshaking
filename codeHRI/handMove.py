#! /usr/bin/env python3
__author__ = "Zhicheng Yang"
__copyright__ = "Copyright 2021, Technical University of Darmstadt"
__credits__ = ["Zhicheng Yang"]
__license__ = "GNU GPL v3.0"
__version__ = "1.0.6"
__maintainer__ = "Zhicheng Yang"
__email__ = "zhicheng.yang@stud.tu-darmstadt.de"
__status__ = "Experimental"
# ros imports
import rospy

# festo imports
from festo_phand_msgs.msg import SimpleFluidPressures

pressure_pub = rospy.Publisher("/festo/phand/set_pressures", SimpleFluidPressures, queue_size=1)
msg = SimpleFluidPressures()

if __name__ == "__main__":
    
    rospy.init_node("festo_hand_move")
    rate = rospy.Rate(10)
    
    msg.values = [100000.0] * 12
    msg.values[5] = 250000

    while not rospy.is_shutdown():

        # Publish the pressure values
        for i in range(100000, 600001, 10000):
            msg.values[1] = i
            msg.values[3] = i
            msg.values[4] = i
            msg.values[6] = i
            msg.values[8] = i
            msg.values[10] = i
            msg.values[11] = i

            pressure_pub.publish(msg)
            rospy.loginfo("data for close:%s", msg.values)
            rate.sleep()

        for i in range(600000, 99999, -10000):
            msg.values[1] = i
            msg.values[3] = i
            msg.values[4] = i
            msg.values[6] = i
            msg.values[8] = i
            msg.values[10] = i
            msg.values[11] = i

            pressure_pub.publish(msg)
            rospy.loginfo("data for open:%s", msg.values)
            rate.sleep()