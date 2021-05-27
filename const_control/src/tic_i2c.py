#!/usr/bin/env python
from smbus2 import SMBus, i2c_msg
from time import time, sleep

class TicI2C(object):
    """ Handles communication with a tic motor controller/driver. """
    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
 
    def exit_safe_start(self):
        """ Sends the "Exit safe start" command. """
        command = [0x83]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)
    
    def set_target_position(self, target):
        """ Sets the target position. """
        command = [0xE0,
          target >> 0 & 0xFF,
          target >> 8 & 0xFF,
          target >> 16 & 0xFF,
          target >> 24 & 0xFF]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)
 
    def set_max_speed(self, target):
        """ Sets the max speed temporarily. """
        command = [0xE6,
          target >> 0 & 0xFF,
          target >> 8 & 0xFF,
          target >> 16 & 0xFF,
          target >> 24 & 0xFF]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    def set_max_accel(self, target):
        """ Sets the max acceleration temporarily. """
        command = [0xEA,
          target >> 0 & 0xFF,
          target >> 8 & 0xFF,
          target >> 16 & 0xFF,
          target >> 24 & 0xFF]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    def set_current_limit(self, target):
        """ Sets the current limit (in milliamps). """
        value = int(target / 71.615) & 0x7f
        if value > 55:
            value = 55
        #print("setting current limit to {:.2}mA".format(value*71.615))
        command = [0x91, value]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    def set_step_mode(self, microsteps=1):
        """ Sets microstepping according to denominator of 1/1, 1/2, 1/4, or 1/8 temporarily """
        mode = 0 # full stepping
        if microsteps == 2:
            mode = 1
        elif microsteps == 4:
            mode = 2
        elif microsteps == 8:
            mode = 3
        command = [0x94, mode]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    def go_home(self, direction=0): # 0: negative direction, 1: positive direction homing
        """ Starts the homing procedure controlled by the tic """
        command = [0x97, direction]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    def is_homing(self):
        """ True while homing procedure in progress """
        b = self.get_variables(0x01, 1)
        homing_active = (b[0] & 0x10) != 0
        #print("{}: homing active".format(homing_active))
        return homing_active

    def is_certain(self):
        """ True after homing completed and before any error """
        b = self.get_variables(0x01, 1)
        pos_certain = (b[0] & 0x02) == 0
        return pos_certain

    def get_variables(self, offset, length):
        """ Gets one or more variables held by the Tic. """
        write = i2c_msg.write(self.address, [0xA1, offset])
        read = i2c_msg.read(self.address, length)
        self.bus.i2c_rdwr(write, read)
        return list(read)
 
    def get_state(self):
        """ Gets Operation State where 0:reset, 2:de-nrg, 4:softerr, 6:wait, 8:startup, 10:normal """
        b = self.get_variables(0x00, 1)
        return b[0]

    def get_error(self):
        """ get error bitmap, see https://www.pololu.com/docs/0J71/all for details """
        b = self.get_variables(0x02, 2)
        return b[0]

    def get_current_position(self):
        """ Gets the "Current position" variable from the Tic. """
        b = self.get_variables(0x22, 4)
        position = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24)
        if position >= (1 << 31):
            position -= (1 << 32)
        return position

# stand alone test case/example: moves motor on tic at given bus and address back and forth
if __name__ == "__main__":

    # Open a handle to "/dev/i2c-1", representing the I2C bus.
    bus = SMBus(1)
     
    # Select the I2C address of the Tic (the device number).
    address = 0x03
     
    tic = TicI2C(bus, address)
     
    tic_state = tic.get_state()
    print("state: {}".format(tic_state))
    
    tic_error = tic.get_error()
    print("pre-start error: {}".format(format(tic_error, 'b')))
    
    position = tic.get_current_position()
    print("Current position is {}.".format(position))
     
    
    tic.exit_safe_start()
    
    tic_error = tic.get_error()
    print("post-start error: {}".format(format(tic_error, 'b')))
    
    travel = target = 100000
    i = 0
    print("Setting target position to {}.".format(target));
    while i < 10:
        tic.set_target_position(target)
        sleep(1)
        position = tic.get_current_position()
        print("updated position: {}".format(position))
        if abs(position - target) < abs(travel/100.0):
            i += 1
            if target == travel:
                target = 0
            else:
                target = travel
            print("Setting target position to {}.".format(target));
   
