#!/usr/bin/env python
import numpy as np
import rospy, rospkg, yaml
import math
from arm_kinematics.msg import Pose3D
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class GridViz(object):
    def __init__(self):
        self.can_pub = rospy.Publisher("can_viz", Marker, queue_size=5)
        self.can_center_pub = rospy.Publisher("center_viz", Marker, queue_size=5)
        self.complete_can_sub = rospy.Subscriber("completed_cans", Pose3D, self.appendCompletedCans)
        self.completed_can_pub = rospy.Publisher("completed_cans_viz", Marker, queue_size=5)
        self.completed_cans = []

    def appendCompletedCans(self, data):
        self.completed_cans.append(data)

    def visualizeCompletedCans(self):
        cans = Marker()
        cans.header.frame_id = "grid"
        cans.type = cans.CUBE_LIST
        cans.action = cans.ADD
        cans.lifetime = rospy.Duration.from_sec(0)                                                                                                                                                                                                                                                                                                                 
        cans.scale.x = 0.0665
        cans.scale.y = 0.0665
        cans.scale.z = 0.122
        cans.color.a = 1.0
        cans.color.r = 0.5
        cans.color.g = 0.5
        cans.color.b = 0.5
        cans.pose.orientation.w = 1.0

        for coord in self.completed_cans:
            origin = (coord.x, coord.y, 0.061, coord.theta) #(x,y,z,theta)
            marker_msgs = self.getCanMsg(origin)
            cans.points.extend(marker_msgs)            
            
        self.completed_can_pub.publish(cans)

    def visualizeStructure(self, display_num):

        rospack = rospkg.RosPack()
        file_path = rospack.get_path('arm_navigation') + "/config/center_coords.yaml"

        cans = Marker()
        cans.header.frame_id = "grid"
        cans.type = cans.CUBE_LIST
        cans.action = cans.ADD
        cans.lifetime = rospy.Duration.from_sec(0)
        cans.scale.x = 0.0665
        cans.scale.y = 0.0665
        cans.scale.z = 0.122
        cans.color.a = 0.5
        cans.color.r = 0.69
        cans.color.g = 0.16
        cans.color.b = 0.2
        cans.pose.orientation.w = 1.0

        with open(file_path) as file:
            data = yaml.load(file)

        for i in range(0, min(display_num, len(data))):
            coord = data[i]
            origin = (float(coord[0]/1000.0), float(coord[1]/1000.0), float(coord[2]/1000.0) + 0.061, coord[4]) #(x,y,z,theta)
            marker_msgs = self.getCanMsg(origin)
            cans.points.extend(marker_msgs)        
            
        self.can_pub.publish(cans)

    
    def getCanMsg(self, coord):

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

        return points

    def visualizeCenterCoords(self):

        rospack = rospkg.RosPack()
        file_path = rospack.get_path('arm_navigation') + "/config/center_coords.yaml"

        centers = Marker()
        centers.header.frame_id = "grid"
        centers.type = centers.SPHERE_LIST
        centers.action = centers.ADD
        centers.lifetime = rospy.Duration.from_sec(0)
        centers.scale.x = 0.025
        centers.scale.y = 0.025
        centers.scale.z = 0.025
        centers.color.a = 1.0
        centers.color.r = 1.0
        centers.color.g = 0.0
        centers.color.b = 0.0
        centers.pose.orientation.w = 1.0

        with open(file_path) as file:
            data = yaml.load(file)

        for coord in data:
            center = Point()
            center.x = float(coord[0]/1000.0)
            center.y = float(coord[1]/1000.0)
            center.z = float(coord[2]/1000.0) + 0.122 - 0.0125
            centers.points.append(center)

        self.can_center_pub.publish(centers)
       


