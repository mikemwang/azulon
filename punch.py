import sys, getopt
import atexit

sys.path.append('.')
import RTIMU
import time
import RPi.GPIO as GPIO
import yaml

class Master:
    """ the overall system controller"""
    def __init__(self, params, rate):
        self.rate = rate
        hardware_params = params['hardware_config']
        movement_params = params['movement_params']

        self.controllers = []
        GPIO.setmode(GPIO.BOARD)
        for limb in hardware_params:
            GPIO.setup(hardware_params[limb]["pin"],
                       GPIO.OUT,
                       initial=GPIO.LOW)
            self.controllers.append(Controller(hardware_params,
                                               movement_params))
    def run(self):
        """ blocking call that runs the system indefinitely """
        while True:
            for controller in self.controllers:
                controller.run()
            self.safety()
            time.sleep(self.rate)

    def safety(self):
        """ monitors that no pin stays high for too long """
        pass
        

class Controller:
    """ individual limb state machine """
    def __init__(self, hardware_params, software_params):
        
        self.imu = IMU(hardware_params['name'],
                       hardware_params['config'])

        self.control_pin = hardware_params['pin']
        self.upper_bound = software_params['high']
        self.lower_bound = software_params['low']
        self.window = software_params['window']
        self.duration = software_params['duration']
        self.cooldown = software_params['cooldown']

        self.x = 0
        self.y = 0
        self.z = 0
        self.state = 0

        self.low_detect_time = 0
        self.fire_begin_time = 0
        self.cooldown = 1
        self.cooldown_start = 0

    def close(self):
        GPIO.output(self.control_pin, GPIO.LOW)

    def release(self):
        GPIO.output(self.control_pin, GPIO.HIGH)

    def update(self):
        vals = self.imu.get_accel()
        self.x = vals[0]
        self.y = vals[1]
        self.z = vals[2]
        self.fired = False

    def run(self):
        self.update()
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


class IMU:
    def __init__(self, name, settings, retry=False):
        """
        initialization function
        args:
            name:     name of this imu
            settings: path to settings file (leave out ini extension)
            retry:    whether to retry connecting to imu
        """
        
        self.name = name
        # attempt to connect to imu, repeatedly if retry is true
        self.location = settings
        self.active = False
        while True:
            self.settings = RTIMU.Settings(self.location) 
            self.imu = RTIMU.RTIMU(self.settings)
            if not self.imu.IMUInit():
                print self.name+"."*(2 if len(self.name) > 72 else 72-len(self.name)) + "OFFLINE"
            else:
                print self.name+"."*(2 if len(self.name) > 73 else 73-len(self.name)) + "ONLINE"
                self.active = True
                break
            if not retry:
                return
            time.sleep(1)
        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)
        self.poll_interval = self.imu.IMUGetPollInterval()/1000.0
        print(self.poll_interval)

        # use this to ensure we don't read too often
        self.last_read_time = 0
        # persistent storage of most recent accel value
        self.accel = [0,0,0]

    def get_accel(self):
        """ reads and returns the accel if self.poll_interval has ellapsed,
            else just returns most recent accel """
        if not self.active:
            return self.name+" IMU OFFLINE"
        if time.time() - self.last_read_time > self.poll_interval:
            if self.imu.IMURead():
                self.accel = self.imu.getIMUData()['accel']
                self.last_read_time = time.time()
        return self.accel

def exit_fn():
    print "Exiting cleanly"
    GPIO.cleanup()


if __name__ == '__main__':
    atexit.register(exit_fn)
    GPIOSetup()
    with open("config/config.yaml", 'r') as stream:
        config = yaml.load(stream)
    #test_imu = IMU( "RIGHT ARM", "./config/right_arm",True )
    system = Master(config)
    system.run()
    #while True:
    #    print(test_imu.get_accel())
    #    time.sleep(0.005)
