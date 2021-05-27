#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool

class macTeleop():
    def __init__(self):
        rospy.init_node('mac_teleop')
        rospy.Subscriber('joy', Joy, self.joyCB)
        rospy.Subscriber('mac_arm/joint/fb', JointState, self.armFBCB)
        self.joint_pub = rospy.Publisher('mac_arm/joint/cmd', \
                                JointState, queue_size=10)
        self.grip_pub = rospy.Publisher('grip', Bool, queue_size=10)
        self.axis_scale = [1,0.1,1,1,1,1,1,1]
        self.axis_offset = [0,0,-1,0,0,-1,0,0]
        self.joint_scale = [0.2, 0.2, 0.2, 0.2]
        self.position = [0,0,0,0]
        self.home = [0,0,0,0]

    def armFBCB(self, msg):
        self.position = msg.position
        print(self.position)

    def joyCB(self, msg):
        js = JointState()
        js.name = ['a0', 'a1', 'a2', 'a3']
        if msg.buttons[0] == 1:
            a = self.position[0] + ((msg.axes[1] + self.axis_offset[1]) \
                              * self.axis_scale[1] * self.joint_scale[0])
            js.position = [a,self.position[1],self.position[2],self.position[3]]
        elif msg.buttons[1] == 1:
            a = self.position[1] + ((msg.axes[1] + self.axis_offset[1]) \
                              * self.axis_scale[1] * self.joint_scale[1])
            js.position = [self.position[0],a,self.position[2],self.position[3]]
        elif msg.buttons[2] == 1:
            a = self.position[2] + ((msg.axes[1] + self.axis_offset[1]) \
                              * self.axis_scale[1] * self.joint_scale[2])
            js.position = [self.position[0],self.position[1],a,self.position[3]]
        elif msg.buttons[3] == 1:
            a = self.position[3] + ((msg.axes[1] + self.axis_offset[1]) \
                              * self.axis_scale[1] * self.joint_scale[3])
            js.position = [self.position[0],self.position[1],self.position[2],a]
        elif msg.buttons[8] == 1:
            js.position = [0,0,0,0]
        elif msg.buttons[9] == 1:
            js.position = self.home
        elif msg.buttons[10] == 1:
            self.home = self.position
        else: 
            js.position = self.position

        self.joint_pub.publish(js)

        if msg.buttons[6] == 1:
            self.grip_pub.publish(False)
        elif msg.buttons[7] == 1:
            self.grip_pub.publish(True)


        
if __name__ == "__main__":
  x = macTeleop()
  rospy.spin()
