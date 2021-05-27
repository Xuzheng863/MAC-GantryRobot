#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import serial
from math import *
from tic_i2c import *
import time

#RPRR configuration
names = ['base', 'lift', 'elbow', 'wrist']
addresses = [0x10, 0x11, 0x12, 0x13]
transmissions = [5, (1.0/6.35), -5, 1]
microsteps = [8, 2, 8, 8]

# after homing, these values result in an arm that is extended straight out
kinematic_home = [0.471+0.10548+pi, -0.26, 2.95, 1.6]

# rotation from home in radians at which end effector will rotate to a equivalent/symmetrical position to avoid self-collision
ee_flip_limit = -1.46


def m2step(meters, microstep, transmission, steps_per_rev=100000):
    """ convert meters to steps for a joint with given parameters. """
    return int(meters*microstep*transmission*steps_per_rev)

def r2step(radians, microstep, transmission, steps_per_rev=200.0):
    """ convert radians to steps for a joint with given parameters. """
    return int((radians/(2.0*pi))*microstep*transmission*steps_per_rev)

def step2m(step, microstep, transmission, steps_per_rev=100000):
    """ convert steps to meters for a joint with given parameters. """
    return (float(step))/(microstep*transmission*steps_per_rev)

def step2r(step, microstep, transmission, steps_per_rev=200.0):
    """ convert steps to radians for a joint with given parameters. """
    return (2*pi*step)/(microstep*transmission*steps_per_rev)


class tic_step_driver():
    def __init__(self):
        rospy.init_node('tic_step_driver')
        # rate at which position feedback will be queried and published
        self.rate = rospy.Rate(50)

        self.i2c_bus = SMBus(1)
        self.base_joint = TicI2C(self.i2c_bus, 0x10)
        self.lift_joint = TicI2C(self.i2c_bus, 0x11)
        self.lbow_joint = TicI2C(self.i2c_bus, 0x12)
        self.rist_joint = TicI2C(self.i2c_bus, 0x13)

        rospy.Subscriber('mac_arm/joint/cmd', JointState, self.jointCmdCB)
        self.fb_pub = rospy.Publisher('mac_arm/joint/fb', JointState, queue_size=10)

        print("Tic Stepper driver running")

        # check to see that all of the tic devices are present
        base_state = self.base_joint.get_state()
        lift_state = self.lift_joint.get_state()
        lbow_state = self.lbow_joint.get_state()
        rist_state = self.rist_joint.get_state()
        print("state 1:{} 2:{} 3:{} 4:{}".format(base_state, lift_state, lbow_state, rist_state))

        self.base_joint.exit_safe_start()
        self.lift_joint.exit_safe_start()
        self.lbow_joint.exit_safe_start()
        self.rist_joint.exit_safe_start()

        base_err = self.base_joint.get_error()
        lift_err = self.lift_joint.get_error()
        lbow_err = self.lbow_joint.get_error()
        rist_err = self.rist_joint.get_error()
        print("err 1:{} 2:{} 3:{} 4:{}".format(base_err, lift_err, lbow_err, rist_err))
        # note tic limits (steps, vel, accel, current) are set by USB

        print("Testing Homing...")
        self.base_joint.go_home()
        self.lift_joint.go_home(1)
        self.lbow_joint.go_home(1)
        self.rist_joint.go_home()

        done = False
        while not rospy.is_shutdown() and not done:
            done = (self.base_joint.is_certain() and self.lift_joint.is_certain() and \
                    self.lbow_joint.is_certain() and self.rist_joint.is_certain())
            self.rate.sleep()

        print("done")
        print ("homing={}, certain={}".format(self.rist_joint.is_homing(), self.rist_joint.is_certain()))

        js = JointState()
        js.name = names
        js.position = [0]*len(names)
        js.position[1] = -1.0*kinematic_home[1]
        js.velocity = [0]*len(names)
        js.effort = [0]*len(names)
        self.jointCmdCB(js)
        self.rate.sleep()

    def jointCmdCB(self, msg):
        converted = [0, 0, 0, 0]
        converter = [r2step, m2step, r2step, r2step]
        # since last joint (ee) is symmetrical, we can flip to avoid limit switch collision
        if msg.position[3] < ee_flip_limit: 
            new_poses = [msg.position[0], msg.position[1], msg.position[2], \
                        msg.position[3]+pi]
            msg.position = new_poses
        elif msg.position[3] > ee_flip_limit+pi:
            new_poses = [msg.position[0], msg.position[1], msg.position[2], \
                        msg.position[3]-pi]
            msg.position = new_poses
        #TODO: add joint limits for base and lbow, also mod2pi

        for axis in range(len(msg.name)):
            converted[axis] = converter[axis](msg.position[axis]+kinematic_home[axis], \
                    microsteps[axis], transmissions[axis]) 
            print("Commanding axis {} to position {}({})"
                    .format(msg.name[axis], converted[axis], msg.position[axis]))
        self.base_joint.exit_safe_start()
        self.base_joint.set_target_position(converted[0])
        self.lift_joint.exit_safe_start()
        self.lift_joint.set_target_position(converted[1])
        self.lbow_joint.exit_safe_start()
        self.lbow_joint.set_target_position(converted[2])
        self.rist_joint.exit_safe_start()
        self.rist_joint.set_target_position(converted[3])


    def run(self): 
        while not rospy.is_shutdown():
            # get current joint positions and publish them
            pos_fb = [step2r(self.base_joint.get_current_position(), microsteps[0], transmissions[0]), \
                      step2m(self.lift_joint.get_current_position(), microsteps[1], transmissions[1]), \
                      step2r(self.lbow_joint.get_current_position(), microsteps[2], transmissions[2]), \
                      step2r(self.rist_joint.get_current_position(), microsteps[3], transmissions[3])]
            js = JointState()
            js.name = names
            js.position = [a-b for a,b in zip(pos_fb, kinematic_home)]
            js.velocity = [0]*len(names)
            js.effort = [0]*len(names)
            self.fb_pub.publish(js)
            self.rate.sleep()


if __name__ == "__main__":
    tic_driver = tic_step_driver()
    tic_driver.run()

