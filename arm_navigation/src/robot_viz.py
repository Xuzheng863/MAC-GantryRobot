#!/usr/bin/env python
import numpy as np
import rospy, rospkg, yaml
from arm_kinematics.msg import Pose3D
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class Robot:
    def __init__(self):
        self.robot_pub = rospy.Publisher("robot_viz", Marker, queue_size=10)
        self.gantry_pub = rospy.Publisher("gantry_viz", Marker, queue_size=10)
        self.block_pub = rospy.Publisher("block_viz", Marker, queue_size=5)
        self.origin = [-0.4, -0.2, 0.0]
        self.height = 0.4
        self.width = 1.0
        self.length = 1.7
        
        self.gantryX = 0.7
        self.gantryY = 0.5
        self.gantryZ = 0.3
        self.gantryLength = 0.43
        self.gantry = self._getGantryPose(self.gantryX, self.gantryY, self.gantryZ)

    def visualizeRobot(self):
        robot_frame = Marker()
        robot_frame.header.frame_id = "grid"
        robot_frame.type = robot_frame.LINE_LIST
        robot_frame.action = robot_frame.ADD
        robot_frame.lifetime = rospy.Duration.from_sec(0)
        robot_frame.pose.orientation.w = 1.0
        robot_frame.scale.x = 0.015
        robot_frame.color.b = 1.0
        robot_frame.color.a = 1.0

        robot_frame.points.extend(self._getRobotFrame())

        self.robot_pub.publish(robot_frame)

    def gantryGoTo(self, x, y, z, t, block=None):
        gantry = Marker()
        gantry.header.frame_id = "grid"
        gantry.type = gantry.LINE_LIST
        gantry.action = gantry.ADD
        gantry.lifetime = rospy.Duration.from_sec(0)
        gantry.pose.orientation.w = 1.0
        gantry.scale.x = 0.02
        gantry.color.r = 1.0
        gantry.color.g = 0.55
        gantry.color.b = 0
        gantry.color.a = 1.0

        upDownTime = 0.5
        frame = int(30 * upDownTime)
        rate = rospy.Rate(30)
        zdiff = 0.33 - self.gantry[2].z
        for i in range(frame):
            self.gantry = self._getGantryPose(self.gantry[2].x, self.gantry[2].y, self.gantry[2].z + zdiff/frame)
            gantry.points = self.gantry
            self.gantry_pub.publish(gantry)
            if block:
                self.spawnBlock([self.gantry[2].x, self.gantry[2].y, self.gantry[2].z + zdiff/frame, block[4]])
            rate.sleep()

        xdiff = x - self.gantry[2].x
        ydiff = y - self.gantry[2].y
        frame = t*30
        for i in range(frame):
            self.gantry = self._getGantryPose(self.gantry[2].x + xdiff/frame, self.gantry[2].y + ydiff/frame, self.gantry[2].z)
            gantry.points = self.gantry
            self.gantry_pub.publish(gantry)
            if block:
                self.spawnBlock([self.gantry[2].x + xdiff/frame, self.gantry[2].y + ydiff/frame, self.gantry[2].z, block[4]])
            rate.sleep()

        zdiff = z - self.gantry[2].z
        frame = int(30 * upDownTime)
        for i in range(frame):
            self.gantry = self._getGantryPose(self.gantry[2].x, self.gantry[2].y, self.gantry[2].z + zdiff/frame)
            gantry.points = self.gantry
            self.gantry_pub.publish(gantry)
            if block:
                self.spawnBlock([self.gantry[2].x, self.gantry[2].y, self.gantry[2].z + zdiff/frame, block[4]])
            rate.sleep()
        return

    def pickUpBlock(self, coord):
        self.gantryGoTo(self.gantryX, self.gantryY, 0.061, 2)
        self.spawnBlock([self.gantryX, self.gantryY, 0.061, coord[4]])
        self.gantryGoTo(float(coord[0]/1000.0), float(coord[1]/1000.0), float(coord[2]/1000.0) + 0.061, 2, coord)

    def spawnBlock(self, coord):
        block = Marker()
        block.header.frame_id = "grid"
        block.type = block.CUBE_LIST
        block.action = block.ADD
        block.lifetime = rospy.Duration.from_sec(0)
        block.scale.x = 0.0665
        block.scale.y = 0.0665
        block.scale.z = 0.122
        block.color.a = 0.5
        block.color.r = 0.69
        block.color.g = 0.16
        block.color.b = 0.2
        block.pose.orientation.w = 1.0

        points = []

        if (coord[3]/90.0)%2 == 0:
            relative_origins = [(0.0, 0.03325),(0.0, -0.03325),(0.0665, -0.03325),
                            (0.0665, 0.03325),(-0.0665, 0.03325),(-0.0665, -0.03325)]
        else:
            relative_origins = [(0.03325, 0.0),(-0.03325, 0.0),(-0.03325, 0.0665),
                            (0.03325, 0.0665),(0.03325, -0.0665),(-0.03325, -0.0665)]

        for origin in relative_origins:

            can = Point()
            can.x = coord[0] + origin[0]
            can.y = coord[1] + origin[1]
            can.z = coord[2]
            points.append(can)

        block.points.extend(points)
        self.block_pub.publish(block)


    def _getGantryPose(self, x, y, z):
        points = []
        # Horizontal bar
        p = Point()
        p.x = x
        p.y = self.origin[1]
        p.z = self.origin[2] + self.height
        points.append(p)
        q = Point()
        q.x = x
        q.y = self.origin[1] + self.width
        q.z = self.origin[2] + self.height
        points.append(q)
        # Vertical Bar
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        points.append(p)
        q = Point()
        q.x = x
        q.y = y
        q.z = z + self.gantryLength
        points.append(q)

        return points

    def _getRobotFrame(self):
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