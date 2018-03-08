import sys, getopt
import atexit

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import RPi.GPIO as GPIO

class Controller:
    def __init__(self, control_pin, upper_bound, lower_bound, window):
        self.jerk_thresh = 1.3
        self.control_pin = control_pin
        self.upper_bound = upper_bound
        self.lower_bound = lower_bound
        self.window = window
        self.x = 0
        self.y = 0
        self.z = 0
        self.state = 0
        self.low_detect_time = 0
        self.fire_duration = 0.2
        self.fire_begin_time = 0
        self.cooldown = 1
        self.cooldown_start = 0
        self.lx = 0
        self.ly = 0
        self.lx = 0

    def update(self, new_accel):
        self.lx = self.x
        self.ly = self.y
        self.lz = self.z

        self.x = new_accel[0]
        self.y = new_accel[1]
        self.z = new_accel[2]
        self.fired = False

    def run(self):
        # awaiting dip on x
        if self.state == 0:
            self.close()
            if self.x < self.lower_bound:
                self.low_detect_time = time.time()
                self.state = 1

        # awaiting high x to indicate finished punch
        elif self.state == 1:
            self.close()
            if time.time() - self.low_detect_time > self.window:
                self.state = 0
            else:
                if self.x > self.upper_bound:
                    self.fire_begin_time = time.time()
                    self.state = 2

        # fire on
        elif self.state == 2:
            if time.time() - self.fire_begin_time > self.fire_duration:
                self.fired = True
                self.close()
                self.cooldown_start = time.time()
                self.state = 3
            else:
                self.release()

        elif self.state == 3:
            if time.time() - self.cooldown_start > self.cooldown:
                self.state = 0


    def close(self):
        GPIO.output(self.control_pin, GPIO.LOW)

    def release(self):
        GPIO.output(self.control_pin, GPIO.HIGH)


left_arm_settings = RTIMU.Settings("right_arm")
left_arm = RTIMU.RTIMU(left_arm_settings)
if not left_arm.IMUInit():
    print "couldn't initialize Left Arm"

left_arm.setSlerpPower(0.02)
left_arm.setGyroEnable(True)
left_arm.setAccelEnable(True)
left_arm.setCompassEnable(True)
left_arm_poll_interval = left_arm.IMUGetPollInterval()/1000.0
left_arm_control_pin = 11
left_arm_controller = Controller(left_arm_control_pin, 1.7, -1.7, 0.1)

right_arm_control_pin = 16


# gpio library setup
def GPIOSetup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(right_arm_control_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(left_arm_control_pin, GPIO.OUT, initial=GPIO.LOW)
    #GPIO.setup(right_arm_control_pin, GPIO.OUT, initial=GPIO.LOW)
    #GPIO.setup(left_leg_control_pin, GPIO.OUT, initial=GPIO.LOW)
    #GPIO.setup(right_leg_control_pin, GPIO.OUT, initial=GPIO.LOW)

def exit_fn():
    print "Exiting cleanly"
    GPIO.cleanup()

atexit.register(exit_fn)

counts = 0
state = 0
param = 1.7

if __name__ == '__main__':
    GPIOSetup()
    while True:
        if left_arm.IMURead():
            x = left_arm.getIMUData()['accel'][0]
            if x < -param and state == 0:
                print("here")
                state = 1
            if x > param and state == 1:
                print("there")
                state = 0
                GPIO.output(right_arm_control_pin, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(right_arm_control_pin, GPIO.LOW)


            #left_arm_controller.update(left_arm.getIMUData()['accel'])
        time.sleep(left_arm_poll_interval)

