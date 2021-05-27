#!/usr/bin/env python
import numpy as np
import rospy, rospkg, yaml, math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from arm_kinematics.msg import Pose3D
import tf2_ros, tf2_geometry_msgs

joint_names = ['base', 'lift', 'elbow', 'wrist']

class InverseKinematics(object):
    def __init__(self):
        self.goal_sub = rospy.Subscriber("/end_effector/goal", Pose3D, self.getInverseKinematics)
        self.pose_sub = rospy.Subscriber("mac_arm/joint/fb", JointState, self.eefPose_CB)
        self.joint_pub = rospy.Publisher('mac_arm/joint/cmd', JointState, queue_size=10)
        self.current_pose = JointState()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.arm_params = self.loadArmParams()

    def loadArmParams(self):
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('arm_kinematics') + "/config/arm_params.yaml"

        with open(file_path, "r") as stream:
            data = yaml.load(stream, Loader=yaml.FullLoader)
        return data

    def eefPose_CB(self, data):
        self.current_pose.header = data.header
        self.current_pose.name = data.name
        self.current_pose.position = data.position
        self.current_pose.velocity = data.velocity
        self.current_pose.effort = data.effort

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
                                       "grid", #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
            trans_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return trans_pose
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("TF ERROR")
            return

    #computes inverse kinematics given a end effector pose to transform (x,y,z,theta) goal to joint positions
    def getInverseKinematics(self, curr_eef_pose):
        eef_pose = self.transformToFrame("base_link", curr_eef_pose)
        if eef_pose == None:
            return

        try:
            trans = self.buffer.lookup_transform("rail","base_link",rospy.Time(0.0),rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("tf error")

        eef_x = float(eef_pose.pose.position.x) + trans.transform.translation.x
        eef_y = float(eef_pose.pose.position.y)
        eef_z = float(eef_pose.pose.position.z)

        print("goal eef pose: " + str((eef_x, eef_y, eef_z)))

        if math.sqrt(eef_x**2 + eef_y**2) > self.arm_params['P_RX'] + self.arm_params['R_RX']:
            print("NOT POSSIBLE")
            return

        prismatic_length = eef_z
        point_length = eef_x**2 + eef_y**2 - self.arm_params['P_RX']**2 - self.arm_params['R_RX']**2
        arm_length = 2*self.arm_params['P_RX']*self.arm_params['R_RX']

        theta2_1 = math.atan2(math.sqrt(1 - (point_length/arm_length)**2), (point_length/arm_length))
        theta2_2 = math.atan2(-math.sqrt(1 - (point_length/arm_length)**2), (point_length/arm_length))

        k1_1 = self.arm_params['P_RX'] + (self.arm_params['R_RX']*math.cos(theta2_1))
        k1_2 = self.arm_params['P_RX'] + (self.arm_params['R_RX']*math.cos(theta2_2))

        k2_1 = self.arm_params['R_RX']*math.sin(theta2_1)
        k2_2 = self.arm_params['R_RX']*math.sin(theta2_2)

        theta1_1 = math.atan2(eef_x, eef_y) - math.atan2(k2_1, k1_1)
        theta1_2 = math.atan2(eef_x, eef_y) - math.atan2(k2_2, k1_2)

        #pick solution based on least distance from current joint_poses
        solution = self.findClosestIKSolution([theta1_1, theta2_1], [theta1_2, theta2_2])

        #calculate angle of end effector
        theta3 = self.calculateEndEffectorAngle(solution, math.radians(curr_eef_pose.theta))

        #moves arm to solution joint positions
        self.move_arm([solution[0], prismatic_length, solution[1], theta3])
        rospy.loginfo("goal of " + str([solution[0], prismatic_length, solution[1], theta3]) + " joint positions was sent")

    def findClosestIKSolution(self, sol_1, sol_2):

        #curr_joint_pose = self.current_pose.position

        #diff_1 = [abs(x - y) for x, y in zip(curr_joint_pose, sol_1)]
        #diff_2 = [abs(x - y) for x, y in zip(curr_joint_pose, sol_2)]

        #if sum(diff_1) < sum(diff_2):
        #    return sol_1
        #elif sum(diff_2) < sum(diff_1):
        #    return sol_2
        #return sol_1
        return sol_2

    def move_arm(self, pos):
        js = JointState()
        js.name = joint_names
        js.position = pos
        self.joint_pub.publish(js)

    def calculateEndEffectorAngle(self, revolute_angles, goal_theta):
        summed_revolute_angles = sum(revolute_angles)
        goal_angle = goal_theta - summed_revolute_angles
        return goal_angle

if __name__ == '__main__':

    rospy.init_node("inverse_kinematics")
    kinematics = InverseKinematics()
    rospy.spin()

        
