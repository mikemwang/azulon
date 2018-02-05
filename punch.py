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


left_arm_settings = RTIMU.Settings("left_arm")
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


right_arm_settings = RTIMU.Settings("right_arm")
right_arm = RTIMU.RTIMU(right_arm_settings)
if (not right_arm.IMUInit()):
    print "couldn't initialize IMU"
right_arm.setSlerpPower(0.02)
right_arm.setGyroEnable(True)
right_arm.setAccelEnable(True)
right_arm.setCompassEnable(True)
right_arm_control_pin = 16
right_arm_controller = Controller(right_arm_control_pin, 1.7, -1.7, 0.1)


left_leg_settings = RTIMU.Settings("left_leg")
left_leg = RTIMU.RTIMU(left_leg_settings)
if (not left_leg.IMUInit()):
    print "couldn't initialize IMU"
left_leg.setSlerpPower(0.02)
left_leg.setGyroEnable(True)
left_leg.setAccelEnable(True)
left_leg.setCompassEnable(True)
left_leg_control_pin = 37
left_leg_controller = Controller(left_leg_control_pin, 2.7, -2.7, 0.1)

right_leg_settings = RTIMU.Settings("right_leg")
right_leg = RTIMU.RTIMU(right_leg_settings)
if (not right_leg.IMUInit()):
    print "couldn't initialize IMU"
right_leg.setSlerpPower(0.02)
right_leg.setGyroEnable(True)
right_leg.setAccelEnable(True)
right_leg.setCompassEnable(True)
right_leg_control_pin = 36
right_leg_controller = Controller(right_leg_control_pin, 2.7, -2.7, 0.1)


# gpio library setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(left_arm_control_pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(right_arm_control_pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(left_leg_control_pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(right_leg_control_pin, GPIO.OUT, initial=GPIO.LOW)

def exit_fn():
    print "Exiting cleanly"
    GPIO.cleanup()

atexit.register(exit_fn)

while True:
    if left_arm.IMURead():
        left_arm_controller.update(left_arm.getIMUData()['accel'])
    if right_arm.IMURead():
        right_arm_controller.update(right_arm.getIMUData()['accel'])
        x = right_arm.getIMUData()['accel']
        print(x)
    if left_leg.IMURead():
        left_leg_controller.update(left_leg.getIMUData()['accel'])
    if right_leg.IMURead():
        right_leg_controller.update(right_leg.getIMUData()['accel'])
    #left_arm_controller.run()

    # reinit the imu if a signal was sent to it
    right_arm_controller.run()
    if right_arm_controller.fired:
        right_arm_controller.fired = False
        right_arm_settings = RTIMU.Settings("right_arm")
        right_arm = RTIMU.RTIMU(right_arm_settings)
        if (not right_arm.IMUInit()):
            print "couldn't initialize IMU"
        right_arm.setSlerpPower(0.02)
        right_arm.setGyroEnable(True)
        right_arm.setAccelEnable(True)
        right_arm.setCompassEnable(True)
    #left_leg_controller.run()
    #right_leg_controller.run()
    time.sleep(left_arm_poll_interval)

