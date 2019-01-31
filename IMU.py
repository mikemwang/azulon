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
                print(self.name+"."*(2 if len(self.name) > 72 else 72-len(self.name)) + "OFFLINE")
            else:
                print(self.name+"."*(2 if len(self.name) > 73 else 73-len(self.name)) + "ONLINE")
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
        self.update_accel()
        accel_container[0] = self.accel[0]
        accel_container[1] = self.accel[1]
        accel_container[2] = self.accel[2]
        return self.is_alive()

    def is_alive(self):
        """
        check if the IMU is still running properly
        """
        return self.active and self.got_init_read

    def update_accel(self, array):
        """ 
        updates the acceleration readings
        does not update if it is read before self.poll_interval has ellapsed
        imu is dead if imu.IMURead() returns false
        """
        #if (self.got_init_read and not self.active):
        #    #TODO error handling code here
        #    print("here")
        #    return False
        while True:
            try:
                if (time.time() - self.last_read_time) > self.poll_interval:
                    if (self.got_init_read and not self.active):
                        for i in range(len(array)):
                            array[i] = 0
                    else:
                        if self.imu.IMURead():
                            self.last_read_time = time.time()
                            self.got_init_read = True
                            self.active = True
                            data = self.imu.getIMUData()['accel']
                            for i in range(len(array)):
                                array[i] = data[i]
                        else:
                            self.active = False
                            for i in range(len(array)):
                                array[i] = 0
            except KeyboardInterrupt:
                print("imu exiting cleanly")
                return
            
    
    def init_accel(self):
        if (time.time() - self.last_read_time) > self.poll_interval:
            if self.imu.IMURead():
                self.last_read_time = time.time()
                self.got_init_read = True
                self.active = True
            else:
                self.active = False

    def wait_until_init(self, timeout=500):
        """
        delay until the imu reads something, or until timeout ms has ellapsed
        """
        t = time.time()
        while not self.got_init_read:
            self.init_accel()
            if (time.time() - t > timeout):
                print("TIMED OUT")
                return False
        return True
