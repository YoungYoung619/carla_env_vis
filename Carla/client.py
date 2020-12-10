"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :


Author：Team Li
"""
import config
import sys
import time
from logger import logger
config.carla_module_init()
try:
    import carla
except:
    ValueError('check carla module...')

class Client():
    def __init__(self, host:str="127.0.0.1", port:int=2000, timeout:int=10):
        self.host = host
        self.port = port
        self.timeout = timeout

    def connection(self)->bool:
        """connect to the carla server
        Return:
            flag denote connect success or fail.
        """
        t0 = time.time()
        while self.timeout > (time.time() - t0):
            try:
                self.client = carla.Client(self.host, self.port)
                self.client.set_timeout(2.)
                logger.info('CARLA %s connected at %s:%d.' % (self.client.get_server_version(), self.host, self.port))
                return True
            except RuntimeError:
                logger.error('CARLA conneted fail...')
                pass
        return False

    def world(self):
        return self.client.get_world()

if __name__ == '__main__':
    client = client()
    client.connection()
    pass