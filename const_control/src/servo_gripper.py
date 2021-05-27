#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import serial

class servo_gripper():
    def __init__(self):
        rospy.init_node('servo_gripper')
        port = rospy.get_param('~port', '/dev/ttyACM0')
        baud = rospy.get_param('~baud', 115200)
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=1.0)
        rospy.Subscriber('grip', Bool, self.gripCB)
        print("Gripper driver running")

    def gripCB(self, msg):
        if msg.data == True:
            self.serial.write(b'c')
            rospy.loginfo('closing')
        else:
            self.serial.write(b'o')
            rospy.loginfo('opening')

if __name__ == "__main__":
    gripper = servo_gripper()
    rospy.spin()
        
