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
    
    def get_accels(self, accel_container):
        accel_container[0] = self.accel[0]
        accel_container[1] = self.accel[1]
        accel_container[2] = self.accel[2]
        return self.is_alive

    def is_alive(self):
        """
        check if the IMU is still running properly
        """
        return self.active and self.got_init_read

    def update_accel(self):
        """ 
        updates the acceleration readings
        does not update if it is read before self.poll_interval has ellapsed
        imu is dead if imu.IMURead() returns false
        """
        if (self.got_init_read and not self.active):
            #TODO error handling code here
            return

        if time.time() - self.last_read_time > self.poll_interval:
            self.last_read_time = time.time()
            if self.imu.IMURead():
                self.got_init_read = True
                self.active = True
                self.accel = self.imu.getIMUData()['accel']
            else:
                self.active = False
    
    def wait_until_init(self, timeout=500):
        """
        delay until the imu reads something, or until timeout ms has ellapsed
        """
        t = time.time()
        while not self.got_init_read:
            self.update_accel()
            if (time.time() - t > timeout):
                print("TIMED OUT")
                return False
        return True
