import sys, getopt  
import atexit
   
sys.path.append('.')  
import RTIMU  
import os.path  
import time  
import math  
import RPi.GPIO as GPIO

from smbus import SMBus
import time

bus = SMBus(1)

 
SETTINGS_FILE = "RTIMULib"  
LIGHTER_PIN = 8
CHAMBER_PIN = 10
ACCEL_THRESHOLD = 1.5
 
s = RTIMU.Settings("settings_1")  
s2 = RTIMU.Settings("settings_test")
imu = RTIMU.RTIMU(s)  
imu2 = RTIMU.RTIMU(s2)
 
if (not imu.IMUInit()):  
    print "couldn't initialize IMU"

if (not imu2.IMUInit()):  
    print "couldn't initialize IMU2"

bus = SMBus(1)
 
imu.setSlerpPower(0.02)  
imu.setGyroEnable(True)  
imu.setAccelEnable(True)  
imu.setCompassEnable(True)  

imu2.setSlerpPower(0.02)  
imu2.setGyroEnable(True)  
imu2.setAccelEnable(True)  
imu2.setCompassEnable(True)  
 
poll_interval = imu.IMUGetPollInterval()  
poll_interval2 = imu2.IMUGetPollInterval()  

# gpio library setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(LIGHTER_PIN, GPIO.OUT)
GPIO.setup(CHAMBER_PIN, GPIO.OUT)

def exit_fn():
    print "Exiting cleanly"
    GPIO.cleanup()

atexit.register(exit_fn)
dead_time = 0

def open_and_close(time_delay):
    GPIO.output(LIGHTER_PIN, GPIO.HIGH)
    GPIO.output(CHAMBER_PIN, GPIO.HIGH)
    print GPIO.input(CHAMBER_PIN)
    GPIO.output(CHAMBER_PIN, GPIO.HIGH)
    print GPIO.input(CHAMBER_PIN)
    GPIO.output(CHAMBER_PIN, GPIO.HIGH)
    print GPIO.input(CHAMBER_PIN)

    time.sleep(time_delay)

    GPIO.output(LIGHTER_PIN, GPIO.LOW)
    GPIO.output(CHAMBER_PIN, GPIO.LOW)
    print GPIO.input(CHAMBER_PIN)
 
while True:  

    if imu.IMURead():  
        data = imu.getIMUData()  

        #fields
        fusionPose = data["fusionPose"]  
        gyro = data["gyro"]  
        fusionQPose = data["fusionQPose"]  
        accel = data["accel"]  
        compass = data["compass"]  

        print "accel1: ", accel
        dead_time = time.time()
    else:
        if time.time()-dead_time > 0.1:
            print("resetting")
            imu = RTIMU.RTIMU(s)  
            imu.IMUInit()
            imu.setSlerpPower(0.02)  
            imu.setGyroEnable(True)  
            imu.setAccelEnable(True)  
            imu.setCompassEnable(True)  
            dead_time - time.time()

    #if imu2.IMURead():  
    #    data = imu2.getIMUData()  

    #    #fields
    #    fusionPose = data["fusionPose"]  
    #    gyro = data["gyro"]  
    #    fusionQPose = data["fusionQPose"]  
    #    accel = data["accel"]  
    #    compass = data["compass"]  

    #    print "accel2: ", accel


    #    #if -accel[1] > ACCEL_THRESHOLD:
    #    #     print "FIRE IN THE HOLE"
    #    #     open_and_close(1)
    #    #     time.sleep(0.5)

    time.sleep(poll_interval*1.0/1000.0)  

