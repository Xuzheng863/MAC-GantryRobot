#!/usr/bin/env python
import numpy as np
import sys
import rospy, rospkg, yaml
from arm_kinematics.msg import Pose3D
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from defines import *

class Robot:
    def __init__(self, robot_name):
        self.gantry_pub = rospy.Publisher("gantry_viz_" + robot_name, Marker, queue_size=10)
        self.block_pub = rospy.Publisher("block_viz_" + robot_name, Marker, queue_size=5)
        self.origin = ROBOT_ORIGIN
        self.height = ROBOT_HEIGHT
        self.width = ROBOT_WIDTH
        self.length = ROBOT_LENGTH
        
        self.gantryX, self.gantryY, self.gantryZ = INIT_POS[robot_name]
        self.gantryLength = 0.43
        self.gantry = self._getGantryPose(self.gantryX, self.gantryY, self.gantryZ)

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



if __name__ == "__main__":
    if not len(sys.argv) == 5:
        sys.exit("Invalid Argument! Double Check in launch file\n\
                  Usage: agrs=\{gantry_name, gentry_pose}")
    _, robot_name, file_path, _, _ = sys.argv

    rospy.init_node("gantry_" + robot_name)
    can_pub_to_viz = rospy.Publisher(robot_name, Float32MultiArray, queue_size=10)
    robot = Robot(robot_name)

    # rospack = rospkg.RosPack()
    # file_path = rospack.get_path('arm_navigation') + "/config/center_coords.yaml"
    with open(file_path) as file:
        data = yaml.load(file)

    rospy.loginfo("Robot " + robot_name + " moving.")

    block_num = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if block_num < len(data):
            robot.pickUpBlock(data[block_num])
            can_pub = list(data[block_num])
            can_pub.pop(3)
            can_pub_to_viz.publish(Float32MultiArray(data=can_pub))
            block_num += 1
        else:
            rospy.loginfo(robot_name + " Build List Succesfull")
            rate.sleep()