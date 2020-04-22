import logging
import sys
import multiprocessing


class Config(dict):
    def __init__(self, logger=None):
        self["authkey"] = b"0123456789"
        #self["message_bus_address"] = ('127.0.0.1', 50000)
        self["message_bus_address"] = ('192.168.1.16', 50000)
        self["message_bus_address_remote"] = ('192.168.1.16', 50000)
        self["message_bus_max_queue_size"] = 50

        self["initial_goal"] = "START" 
        self["serial_port_a"] = "/dev/ttyACM0"
        self["serial_port_a_baud_rate"] = 9600 # 19200  115200
        
        self['lidar_port_name'] = '/dev/ttyUSB0'

        self['map_grid_size'] = (40,40)  # 20 20 

        #slam
        self['slam.set_boundaries'] = True

        self.log = None
        if logger is None:
            self.log = self.init_logger() 
        

    def init_logger(self):
        #log = multiprocessing.get_logger()
        logger = multiprocessing.log_to_stderr()
        logger.setLevel(multiprocessing.SUBDEBUG)  # .INFO DEBUG # 

        logger.info("configured loggers")
        return logger

    def close(self):
        if (self.log != None):
            self.log.info("closing logging")
            for handler in self.log.handlers:
                handler.close()
                self.log.removeFilter(handler)
