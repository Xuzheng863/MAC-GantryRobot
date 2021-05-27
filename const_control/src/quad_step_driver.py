#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import serial
from math import *

#RPRR configuration
names = ['base', 'lift', 'elbow', 'wrist']
transmissions = [-5, -1, -5, -1]
microsteps = [1, 1, 8, 8]
cmd_set_microsteps = b'/1j1,1,8,8R\r'
cmd_set_velocity = b'/1V100,500,500,200R\r' # microsteps/second (last 2 can go faster)
cmd_set_acceleration = b'/1L1,100000,1,1R\r' # microsteps/second^2
cmd_set_move_current = b'/1mR10,100,100,30R\r' # % of 2A (up to 100%)
cmd_set_hold_current = b'/1h10,30,30,20R\r'   # % pf 2A (up to 50%)

qry_hello = b'/1&\r'
qry_pos = b'/1?aA\r'


def m2step(meters, microstep, transmission, steps_per_rev=100000):
    return int(meters*microstep*transmission*steps_per_rev)

def r2step(radians, microstep, transmission, steps_per_rev=200.0):
    return int((radians/(2.0*pi))*microstep*transmission*steps_per_rev)

def step2m(step, microstep, transmission, steps_per_rev=100000):
    return (float(step))/(microstep*transmission*steps_per_rev)

def step2r(step, microstep, transmission, steps_per_rev=200.0):
    return (2*pi*step)/(microstep*transmission*steps_per_rev)


class quad_step_driver():
    def __init__(self):
        rospy.init_node('quad_step_driver')
        self.rate = rospy.Rate(50)

        port = rospy.get_param('~port', '/dev/ttyUSB0')
        #baud = rospy.get_param('~baud', 9600)
        self.serial = serial.Serial(port=port,  timeout=0.5) #baudrate=baud
        self.softmutex = 0

        rospy.Subscriber('mac_arm/joint/cmd', JointState, self.jointCmdCB)
        self.fb_pub = rospy.Publisher('mac_arm/joint/fb', JointState, queue_size=10)


        print("Quad Stepper driver running")
        self.serial.write(qry_hello)
        r = self.serial.readline()
        print(r[17:-3])
        print("Setting microsteps, velocity, acceleration, voltage, and current...")
        self.serial.write(cmd_set_velocity)
        r = self.serial.readline()
        print ("vel: {}".format(r[2:-2]))

        self.serial.write(cmd_set_move_current)
        r = self.serial.readline()
        print ("m_c: {}".format(r[2:-2]))

        self.serial.write(cmd_set_hold_current)
        r = self.serial.readline()
        print ("h_c: {}".format(r[2:-2]))

        self.serial.write(cmd_set_microsteps)
        r = self.serial.readline()
        print ("stp: {}".format(r[2:-2]))

        print("Skipping Homing...")


    def jointCmdCB(self, msg):
        converted = [0, 0, 0, 0]
        converter = [r2step, m2step, r2step, r2step]
        for axis in range(len(msg.name)):
            converted[axis] = converter[axis](msg.position[axis], \
                    microsteps[axis], transmissions[axis])
            print("Commanding axis {} to position {}({})"
                    .format(msg.name[axis], converted[axis], msg.position[axis]))
        while self.softmutex != 0:
            #print "waiting in cb"
            rospy.sleep(0.001)
        self.softmutex = 1
        self.serial.write(b'/1A{},{},{},{}R\r'
                .format(converted[0], converted[1], converted[2], converted[3]))
        response = self.serial.readline()
        self.softmutex = 0
        print("response: {}".format(response[2:-2]))


    def run(self): 
        while not rospy.is_shutdown():
            while self.softmutex != 0:
                #print "waiting in run"
                rospy.sleep(0.001)
            self.softmutex = 1
            self.serial.write(qry_pos)
            response = self.serial.readline()
            self.softmutex = 0
            if '`' in response:
                pos_steps = response.strip('\x03\r\n').split('`')[1].split(',')
                converter = [step2r, step2m, step2r, step2r]
                pos_si = [0]*len(names)
                for axis in range(len(converter)):
                    pos_si[axis] = converter[axis](int(pos_steps[axis]), \
                            microsteps[axis], transmissions[axis])
                js = JointState()
                js.name = names
                js.position = pos_si
                js.velocity = [0]*len(names)
                js.effort = [0]*len(names)
                self.fb_pub.publish(js)
            self.rate.sleep()


if __name__ == "__main__":
    qs_driver = quad_step_driver()
    qs_driver.run()

