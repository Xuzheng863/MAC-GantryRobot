#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import serial
from math import *
from tic_i2c import *
import time

#RPRR configuration
names = ['rail']
addresses = [0x14]
transmissions = [1]
microsteps = [2]

def m2step(meters, microstep, transmission, steps_per_rev=40000):
    """ convert meters to steps for a joint with given parameters. """
    return int(meters*microstep*transmission*steps_per_rev)

def r2step(radians, microstep, transmission, steps_per_rev=200.0):
    """ convert radians to steps for a joint with given parameters. """
    return int((radians/(2.0*pi))*microstep*transmission*steps_per_rev)

def step2m(step, microstep, transmission, steps_per_rev=40000):
    """ convert steps to meters for a joint with given parameters. """
    return (float(step))/(microstep*transmission*steps_per_rev)

def step2r(step, microstep, transmission, steps_per_rev=200.0):
    """ convert steps to radians for a joint with given parameters. """
    return (2*pi*step)/(microstep*transmission*steps_per_rev)


class tic_step_driver():
    def __init__(self):
        rospy.init_node('rail_driver')
        # rate at which position feedback will be queried and published
        self.rate = rospy.Rate(50)

        self.i2c_bus = SMBus(1)
        self.rail_joint = TicI2C(self.i2c_bus, 0x14)

        rospy.Subscriber('mac_arm/rail/cmd', JointState, self.jointCmdCB)
        self.fb_pub = rospy.Publisher('mac_arm/rail/fb', JointState, queue_size=10)

        print("Tic RAIL Stepper driver running")

        # check to see that all of the tic devices are present
        rail_state = self.rail_joint.get_state()
        print("state:{}".format(rail_state))

        self.rail_joint.exit_safe_start()
        rail_err = self.rail_joint.get_error()
        print("err {}".format(rail_err))
        # note tic limits (steps, vel, accel, current) are set by USB

        print("Homing...")
        self.rail_joint.go_home()

        done = False
        while not rospy.is_shutdown() and not done:
            done = (self.rail_joint.is_certain())
            self.rate.sleep()

        print("done")

        #js = JointState()
        #js.name = names
        #js.position = [0]*len(names)
        #js.position[1] = -1.0*kinematic_home[1]
        #js.velocity = [0]*len(names)
        #js.effort = [0]*len(names)
        #self.jointCmdCB(js)
        #self.rate.sleep()

    def jointCmdCB(self, msg):
        converted = [0]
        converter = [m2step]

        for axis in range(len(msg.name)):
            converted[axis] = converter[axis](msg.position[axis], \
                    microsteps[axis], transmissions[axis]) 
            print("Commanding axis {} to position {}({})"
                    .format(msg.name[axis], converted[axis], msg.position[axis]))
        self.rail_joint.exit_safe_start()
        self.rail_joint.set_target_position(converted[0])


    def run(self): 
        while not rospy.is_shutdown():
            # get current joint positions and publish them
            pos_fb = [step2m(self.rail_joint.get_current_position(), \
                                microsteps[0], transmissions[0])]
            js = JointState()
            js.name = names
            js.position = [pos_fb]
            js.velocity = [0]*len(names)
            js.effort = [0]*len(names)
            self.fb_pub.publish(js)
            self.rate.sleep()


if __name__ == "__main__":
    tic_driver = tic_step_driver()
    tic_driver.run()

