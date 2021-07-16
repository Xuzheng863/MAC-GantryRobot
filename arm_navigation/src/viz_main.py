#!/usr/bin/env python
import sys
import rospy, rospkg, yaml
from grid_viz import GridViz
from robot_viz import Robot

def main():
    rospy.init_node("grid_viz")
    viz = GridViz()
    robot = Robot()

    rate = rospy.Rate(5)
    display_num = 1
    block_num = 0

    rospack = rospkg.RosPack()
    file_path = rospack.get_path('arm_navigation') + "/config/center_coords.yaml"
    with open(file_path) as file:
        data = yaml.load(file)

    rospy.loginfo("moving")

    while not rospy.is_shutdown():
        if block_num < len(data):
            # robot.visualizeRobot()
            robot.pickUpBlock(data[block_num])
            viz.visualizeStructure(display_num)
            display_num += 1
            block_num += 1

if __name__ == "__main__":
    main()
