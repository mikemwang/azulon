import sys, getopt
import atexit

sys.path.append('.')
import time
import RPi.GPIO as GPIO

import IMU

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





# gpio library setup
def GPIOSetup():
    GPIO.setmode(GPIO.BOARD)
    #GPIO.setup(right_arm_control_pin, GPIO.OUT, initial=GPIO.LOW)
    #GPIO.setup(left_arm_control_pin, GPIO.OUT, initial=GPIO.LOW)
    #GPIO.setup(right_arm_control_pin, GPIO.OUT, initial=GPIO.LOW)
    #GPIO.setup(left_leg_control_pin, GPIO.OUT, initial=GPIO.LOW)
    #GPIO.setup(right_leg_control_pin, GPIO.OUT, initial=GPIO.LOW)

def exit_fn():
    print "Exiting cleanly"
    GPIO.cleanup()

atexit.register(exit_fn)

class TestClass:
    def __init__(self, imu):
        self.imu = imu
        self.data = [0,0,0]

    def get(self):
        self.imu.update_accel()
        if not self.imu.get_accels(self.data):
            print("BAD")
        else:
            print(self.data)

    

if __name__ == '__main__':
    GPIOSetup()
    test_imu = IMU.IMU( "LEFT ARM", "./config/left_arm",True )
    test_class = TestClass(test_imu)
    
    while True:
        test_class.get()
        time.sleep(0.005)
