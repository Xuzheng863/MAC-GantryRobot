#!/usr/bin/env python
import rospy
import sys
from arm_kinematics.msg import Pose3D
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from defines import *

class RobotFrame:
    def __init__(self, height, width, length):
        self.robot_pub = rospy.Publisher("robot_viz", Marker, queue_size=10)
        self.origin = ROBOT_ORIGIN
        self.height = height
        self.width = width
        self.length = length

    def visualizeFrame(self):
        robot_frame = Marker()
        robot_frame.header.frame_id = "grid"
        robot_frame.type = robot_frame.LINE_LIST
        robot_frame.action = robot_frame.ADD
        robot_frame.lifetime = rospy.Duration.from_sec(0)
        robot_frame.pose.orientation.w = 1.0
        robot_frame.scale.x = 0.015
        robot_frame.color.b = 1.0
        robot_frame.color.a = 1.0

        robot_frame.points.extend(self.__getRobotFrame())

        self.robot_pub.publish(robot_frame)

    def __getRobotFrame(self):
        points = []
        # lower bottom
        p = Point()
        p.x = self.origin[0]
        p.y = self.origin[1]
        p.z = self.origin[2]
        points.append(p)
        q = Point()
        q.x = self.origin[0] + self.length
        q.y = self.origin[1]
        q.z = self.origin[2]
        points.append(q)
        # lower up
        p = Point()
        p.x = self.origin[0]
        p.y = self.origin[1] + self.width
        p.z = self.origin[2]
        points.append(p)
        q = Point()
        q.x = self.origin[0] + self.length
        q.y = self.origin[1] + self.width
        q.z = self.origin[2]
        points.append(q)
        # upper bottom
        p = Point()
        p.x = self.origin[0]
        p.y = self.origin[1]
        p.z = self.origin[2] + self.height
        points.append(p)
        q = Point()
        q.x = self.origin[0] + self.length
        q.y = self.origin[1]
        q.z = self.origin[2] + self.height
        points.append(q)
        # upper up
        p = Point()
        p.x = self.origin[0]
        p.y = self.origin[1] + self.width
        p.z = self.origin[2] + self.height
        points.append(p)
        q = Point()
        q.x = self.origin[0] + self.length
        q.y = self.origin[1] + self.width
        q.z = self.origin[2] + self.height
        points.append(q)

        return points


if __name__ == "__main__":
    rospy.init_node("robot_frame_viz")

    rate = rospy.Rate(1)
    robot_frame = RobotFrame(ROBOT_HEIGHT, ROBOT_WIDTH, ROBOT_LENGTH)

    rospy.loginfo("Publishing gantry frame to rviz")
    while True:
        robot_frame.visualizeFrame()
        rate.sleep()