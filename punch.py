import sys, getopt
import atexit
from multiprocessing import Process, Queue, Array

sys.path.append('.')
import time
import RPi.GPIO as GPIO
import signal

import IMU

class ArmFSM:
    def __init__(self, control_pin, high, low, window, duration, cooldown, imu):
        self.control_pin = control_pin
        self.high_thresh = high
        self.low_thresh = low
        self.window_t = window
        self.duration_t = duration
        self.cooldown_t = cooldown
        self.imu = imu

        self.accels = [0,0,0]
        
        self.last_fire_t = time.time()
    
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

def exit_fn(signal=None, frame=None):
    print("Exiting cleanly")
    GPIO.cleanup()

atexit.register(exit_fn)

class AllProcesses:
    def __init__(self, imu_controller_pairs):
        procs = []
        for pair in imu_controller_pairs:
            arr = Array('d', range(3))

            p = Process(target=pair['imu'].update_accel, args=(arr,))
            p.start()
            procs.append(p)

            r = Process(target=pair['controller'].update, args=(arr,))
            r.start()
            procs.append(r)

        for proc in procs:
            proc.join()

class Controller:
    def __init__(self):
        self.inited = True
        self.last_update_t = time.time()
        self.accels = None
        self.on = False

    def update(self, array):
        GPIO.setup(16, GPIO.OUT, initial=GPIO.LOW)
        while True:
            try:
                if time.time() - self.last_update_t > 3:
                    if not self.on: 
                        GPIO.output(16, GPIO.HIGH)
                        self.on = True
                    else:
                        GPIO.output(16, GPIO.LOW)
                        self.on = False

                    self.last_update_t = time.time()
                    self.accels = array[:]
                    print(self.accels)
            except KeyboardInterrupt:
                print("controller exiting cleanly")
                GPIO.cleanup()
                return
    

if __name__ == '__main__':
    GPIOSetup()
    test_imu = IMU.IMU( "LEFT ARM", "./config/left_arm",True )

    test_controller = Controller()
    AllProcesses([{'imu': test_imu, 'controller': test_controller}])
    #dead = False
    #while not dead:
    #    print("here")
    #    if not dead:
    #        try:
    #            print "Waiting 10 seconds"
    #            time.sleep(10)

    #        except KeyboardInterrupt:
    #            dead = True
    #            print "Caught KeyboardInterrupt, terminating workers"
    
    #while True:
    #    #inp = raw_input("press enter")
    #    print(test_imu.imu.IMURead())
    #    time.sleep(0.002)
    #    #print(test_imu.update_accel())
    #    #print(test_imu.accel)