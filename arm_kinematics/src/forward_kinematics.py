#!/usr/bin/env python
# rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 test_parent test_child
import numpy as np
import rospy, rospkg, yaml, math
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ForwardKinematics(object):
    def __init__(self):
        self.arm_params = self.loadArmParams()
        self.joint_sub = rospy.Subscriber("mac_arm/joint/fb", JointState, self.pubForwardKinematics)
        self.eef_pub = rospy.Publisher("end_effector/pose", PoseStamped, queue_size=1)
        self.goal_sub = rospy.Subscriber("mac_arm/joint/cmd", JointState, self.pubGoalPose)
        self.goal_sub = rospy.Subscriber("mac_arm/joint/fb", JointState, self.pubCurrPose)
        self.goal_pub = rospy.Publisher("end_effector/goal_viz", Marker, queue_size=10)
        self.curr_pub = rospy.Publisher("end_effector/curr_viz", Marker, queue_size=10)

    def loadArmParams(self):
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('arm_kinematics') + "/config/arm_params.yaml"

        with open(file_path, "r") as stream:
            data = yaml.load(stream, Loader=yaml.FullLoader)
        return data

    def pubGoalPose(self, data):
        goal_marker_msg = self.getVisualMsg(data, (0.75,0,0))
        self.goal_pub.publish(goal_marker_msg)

    def pubCurrPose(self, data):
        goal_marker_msg = self.getVisualMsg(data, (0,0,1))
        self.curr_pub.publish(goal_marker_msg)

    def pubForwardKinematics(self, data):
        eef_pose = PoseStamped()
        eef_pose.header.frame_id = "base_link"

        fwd_kinematics = self.getForwardKinematics(data)
        eef = fwd_kinematics[3]
        eef_pose.pose.position.x = eef[0]
        eef_pose.pose.position.y = eef[1]
        eef_pose.pose.position.z = eef[2]
        eef_orientation = fwd_kinematics[4]
        eef_pose.pose.orientation.x = eef_orientation[0]
        eef_pose.pose.orientation.y = eef_orientation[1]
        eef_pose.pose.orientation.z = eef_orientation[2]
        eef_pose.pose.orientation.w = eef_orientation[3]
        self.eef_pub.publish(eef_pose)

    def getForwardKinematics(self, data):
        
        joint_info = data.position #revolute1, prismatic1, revolute2, revolute
        theta_0, length_0, theta_2, theta_3 = [x for x in joint_info] 

        #base assumed to be fixed at (0,0,0)
        shoulder_x = 0.0
        shoulder_y = 0.0
        shoulder_z = length_0

        elbow_x = self.arm_params['P_RX']*math.sin(theta_0)
        elbow_y = self.arm_params['P_RX']*math.cos(theta_0)
        elbow_z = length_0

        wrist_x = elbow_x + self.arm_params['R_RX']*math.sin(theta_0 + theta_2)
        wrist_y = elbow_y + self.arm_params['R_RX']*math.cos(theta_0 + theta_2)
        wrist_z = length_0

        eef_x = wrist_x
        eef_y = wrist_y 
        eef_z = length_0

        eef_theta = theta_0 + theta_2
        goal_quaternion = quaternion_from_euler(0, 0, math.radians(eef_theta))

        return [(shoulder_x, shoulder_y, shoulder_z), (elbow_x, elbow_y, elbow_z), 
                (wrist_x, wrist_y, wrist_z), (eef_x, eef_y, eef_z), goal_quaternion]

    def getVisualMsg(self, pose_data, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.orientation.w = 1.0
        marker.points = []

        fwd_kinematics = self.getForwardKinematics(pose_data)
        for k in range(4):
            point = Point()
            point.x = fwd_kinematics[k][0]
            point.y = fwd_kinematics[k][1]
            point.z = fwd_kinematics[k][2]
            marker.points.append(point)

        return marker

if __name__ == '__main__':

    rospy.init_node("forward_kinematics")
    kinematics = ForwardKinematics()
    rospy.spin()
