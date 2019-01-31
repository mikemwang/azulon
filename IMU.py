import time
import RTIMU
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

        # use this to ensure we don't read too often
        self.last_read_time = 0

        # persistent storage of most recent accel value
        self.accel = [0,0,0]
        
        # initialize the last read time
        self.last_read_time = time.time()
        self.got_init_read = False
        self.active = self.wait_until_init()

    def update_accel(self):
        """ 
        reads and returns the accel if self.poll_interval has ellapsed,
        else just returns most recent accel 
        """
        if time.time() - self.last_read_time > self.poll_interval:
            self.last_read_time = time.time()
            if self.imu.IMURead():
                self.got_init_read = True
                self.active = True
                self.accel = self.imu.getIMUData()['accel']
            else:
                self.active = False
    
    def wait_until_init(self, timeout=500):
        t = time.time()
        while not self.got_init_read:
            self.update_accel()
            if (time.time() - t > timeout):
                print("TIMED OUT")
                return False
        return True
