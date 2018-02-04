import sys, getopt  
import atexit
   
sys.path.append('.')  
import RTIMU  
import os.path  
import time  
import math  
import RPi.GPIO as GPIO

class Accelerometer:
    def __init__(self, config):
        settings = RTIMU.Settings(config)
        self.imu = RTIMU.RTIMU(settings)
        if (not self.imu.IMUInit()):  
            print "couldn't initialize IMU"
        self.imu.setSlerpPower(0.02)  
        self.imu.setGyroEnable(True)  
        self.imu.setAccelEnable(True)  
        self.imu.setCompassEnable(True)  
        self.poll_interval = self.imu.IMUGetPollInterval()/1000.0  
        self.last_data = None

    def read(self):
        if self.imu.IMURead():
            data = self.imu.getIMUData()['accel']
            self.last_data = data
            return data
        else:
            return self.last_data
         

left_arm_settings = RTIMU.Settings("left_arm")
left_arm = RTIMU.RTIMU(left_arm_settings)
if (not left_arm.IMUInit()):
    print "couldn't initialize IMU"
left_arm.setSlerpPower(0.02)  
left_arm.setGyroEnable(True)  
left_arm.setAccelEnable(True)  
left_arm.setCompassEnable(True)  
left_arm_poll_interval = left_arm.IMUGetPollInterval()/1000.0  


right_arm_settings = RTIMU.Settings("right_arm")
right_arm = RTIMU.RTIMU(right_arm_settings)
if (not right_arm.IMUInit()):  
    print "couldn't initialize IMU"
right_arm.setSlerpPower(0.02)  
right_arm.setGyroEnable(True)  
right_arm.setAccelEnable(True)  
right_arm.setCompassEnable(True)  
 

left_leg_settings = RTIMU.Settings("left_leg")
left_leg = RTIMU.RTIMU(left_leg_settings)
if (not left_leg.IMUInit()):
    print "couldn't initialize IMU"
left_leg.setSlerpPower(0.02)  
left_leg.setGyroEnable(True)  
left_leg.setAccelEnable(True)  
left_leg.setCompassEnable(True)  


right_leg_settings = RTIMU.Settings("right_leg")
right_leg = RTIMU.RTIMU(right_leg_settings)
if (not right_leg.IMUInit()):  
    print "couldn't initialize IMU"
right_leg.setSlerpPower(0.02)  
right_leg.setGyroEnable(True)  
right_leg.setAccelEnable(True)  
right_leg.setCompassEnable(True)  


## gpio library setup GPIO.setmode(GPIO.BOARD)
#GPIO.setup(LIGHTER_PIN, GPIO.OUT)
#GPIO.setup(CHAMBER_PIN, GPIO.OUT)

def exit_fn():
    print "Exiting cleanly"
    #GPIO.cleanup()

atexit.register(exit_fn)

#def open_and_close(time_delay):
#    GPIO.output(LIGHTER_PIN, GPIO.HIGH)
#    GPIO.output(CHAMBER_PIN, GPIO.HIGH)
#    print GPIO.input(CHAMBER_PIN)
#    GPIO.output(CHAMBER_PIN, GPIO.HIGH)
#    print GPIO.input(CHAMBER_PIN)
#    GPIO.output(CHAMBER_PIN, GPIO.HIGH)
#    print GPIO.input(CHAMBER_PIN)
#
#    time.sleep(time_delay)
#
#    GPIO.output(LIGHTER_PIN, GPIO.LOW)
#    GPIO.output(CHAMBER_PIN, GPIO.LOW)
#    print GPIO.input(CHAMBER_PIN)

#left_arm = Accelerometer("left_arm")

 
while True:  
    if left_arm.IMURead():
        print("left_arm", left_arm.getIMUData()['accel'])
    if right_arm.IMURead():
        print("right_arm", right_arm.getIMUData()['accel'])
    if left_leg.IMURead():
        print("left_leg", left_leg.getIMUData()['accel'])
    if right_leg.IMURead():
        print("right_leg", right_leg.getIMUData()['accel'])
    time.sleep(left_arm_poll_interval)

