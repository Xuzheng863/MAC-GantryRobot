#!/usr/bin/env python

import rospy, rospkg, yaml, math
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from arm_kinematics.msg import Pose3D
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
from math import *
import numpy as np
import tf2_ros, tf2_geometry_msgs

# for TESTING
#pickup_pose = [0.09, 0.05, 0.0, 0.0]
#above_pickup_pose = [0.09, 0.05, 0.18, 0.0]
# for TEN
pickup_pose = [0.035, 0.205, 0.11, 0.0]
above_pickup_pose = [0.035, 0.205, 0.25, 0.0]

class Navigator(object):
    def __init__(self):
        self.pose_sub = rospy.Subscriber("end_effector/pose", PoseStamped, self.poseCB)
        self.goal_pub = rospy.Publisher("end_effector/goal", Pose3D, queue_size=10)
        self.complete_pub = rospy.Publisher("completed_cans", Pose3D, queue_size=10)
        self.gripPub = rospy.Publisher('grip', Bool, queue_size=10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.threshold = 0.003 # meters
        self.seat_offset = 0.007
        self.done = False
        self.current_pose = PoseStamped() 
        self.constructStructure()

    def poseCB(self, data):
        self.current_pose.header = data.header
        self.current_pose.pose = data.pose

    def constructStructure(self):
        rospy.loginfo("starting to build structure")
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('arm_navigation') + "/config/center_coords.yaml"

        with open(file_path) as file:
            data = yaml.load(file, Loader=yaml.FullLoader)

        self.done = False
        self.open_gripper()
        can_count = 0
        while can_count < len(data)  and not rospy.is_shutdown():
            eef_goal = data[can_count]
            eef_x, eef_y, eef_z, eef_theta = [eef_goal[i] for i in (0, 1, 2, 4)]
            #self.pickUpBlock()
            # WARNING Y MULTIPLIED BY -1 CAUSE CHARLIE CANT TF2 (or maybe NS cant atan2)
            self.placeBlock([(eef_x/1000.), (1*eef_y/1000.), (eef_z/1000.), eef_theta])
            can_count += 1
        self.done = True

    def pickUpBlock(self):
        self.sendToJointGoal(above_pickup_pose)
        self.sendToJointGoal(pickup_pose)
        self.close_gripper()

    def doneBuildingStructure(self):
        return self.done

    def placeBlock(self, pose):
        z_height = pose[2] -0.03
        z_plus_one = min(pose[2] + 0.15, 0.26)
        #self.sendToJointGoal([pickup_pose[0], pickup_pose[1], z_plus_one, 0.0])
        #self.sendToJointGoal([pose[0], pose[1], z_plus_one, pose[3]])
        #rospy.sleep(0.5)
        #self.sendToJointGoal([pose[0], pose[1], z_height, pose[3]])
        #self.open_gripper()
        #self.seatBlock(pose)
        self.sendToJointGoal([pose[0], pose[1], z_plus_one, pose[3]])
        #self.sendToJointGoal([pickup_pose[0], pickup_pose[1], z_plus_one, 0.0])

    def seatBlock(self, pose):
        self.sendToJointGoal([pose[0]+self.seat_offset, pose[1], pose[2] - 0.035,pose[3]])
        self.sendToJointGoal([pose[0]-self.seat_offset, pose[1], pose[2] - 0.035,pose[3]])
        self.sendToJointGoal([pose[0], pose[1]+self.seat_offset, pose[2] - 0.035,pose[3]])
        self.sendToJointGoal([pose[0], pose[1]-self.seat_offset, pose[2] - 0.035,pose[3]])
        self.sendToJointGoal([pose[0], pose[1], pose[2], pose[3]]) 

    def sendToJointGoal(self, goal_pose):
        goal_eef_pose = Pose3D()
        goal_eef_pose.header.stamp = rospy.Time.now()
        goal_eef_pose.header.frame_id = "grid"
        goal_eef_pose.x = goal_pose[0]
        goal_eef_pose.y = goal_pose[1]
        goal_eef_pose.z = goal_pose[2]
        goal_eef_pose.theta = goal_pose[3]        
        rospy.loginfo("sending arm to " + str(goal_pose))
        self.goal_pub.publish(goal_eef_pose)
        rospy.sleep(0.1)
        rospy.loginfo("sent arm to " + str(goal_pose))
        while not self.isAtGoal(goal_eef_pose) and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("arm has reached " + str(goal_pose))
        
        #add the translation along the x
        try:
            trans = self.buffer.lookup_transform("rail","base_link",rospy.Time(0.0),rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("tf error")

        goal_eef_pose.x = goal_eef_pose.x + trans.transform.translation.x

        self.complete_pub.publish(goal_eef_pose)

    def isAtGoal(self, goal_pose):
        curr_pose = self.current_pose.pose.position
        curr_pose_arr = [curr_pose.x, curr_pose.y, curr_pose.z]
        
        goal_pose_bl = self.transformToFrame("base_link", goal_pose) #transform wrt to (0,0,0)
        if goal_pose_bl == None:
            print("tf error")
            return

        #add the translation along the x
        try:
            trans = self.buffer.lookup_transform("rail","base_link",rospy.Time(0.0),rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

        goal_pose_bl_arr = [goal_pose_bl.pose.position.x+trans.transform.translation.x, goal_pose_bl.pose.position.y, goal_pose_bl.pose.position.z]
        disFromGoal = math.sqrt(sum([(a - b) ** 2 for a, b in zip(curr_pose_arr, goal_pose_bl_arr)]))
        #print("the distance from {} to {} is {}: ".format(curr_pose_arr, goal_pose_bl_arr, disFromGoal))
        
        if disFromGoal < self.threshold:
            return True

        return False

    def open_gripper(self):
        self.gripPub.publish(Bool(False))
        rospy.sleep(1.0)

    def close_gripper(self):
        self.gripPub.publish(Bool(True))
        rospy.sleep(1.0)

    #transforms data into the world frame
    def transformToFrame(self, target_frame, data):
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose.position.x = data.x
        pose_stamped.pose.position.y = data.y
        pose_stamped.pose.position.z = data.z
        goal_quaternion = quaternion_from_euler(0, 0, math.radians(data.theta))
        pose_stamped.pose.orientation.x = goal_quaternion[0]
        pose_stamped.pose.orientation.y = goal_quaternion[1]
        pose_stamped.pose.orientation.z = goal_quaternion[2]
        pose_stamped.pose.orientation.w = goal_quaternion[3]
        try:
            transform = self.buffer.lookup_transform(target_frame,
                                       data.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
            trans_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return trans_pose
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

if __name__ == '__main__':

    rospy.init_node("navigator")
    nav = Navigator()
    complete_can_pub = rospy.Publisher("curr_structure_viz", Marker, queue_size=5)

    while not nav.doneBuildingStructure():
        rospy.sleep(0.1)

    rospy.loginfo("done building structure")
