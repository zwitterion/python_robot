import multiprocessing as mp
from multiprocessing.managers import BaseManager
from config import Config
import logging

from messagebus import MessageBus, MessageBusProxy

class MessageBusManager(BaseManager):
    def get_message_bus(config, loglevel=logging.DEBUG):
        mp.current_process().authkey = config['authkey']
        #mp.log_to_stderr(loglevel)
        MessageBusManager.register('mbus') 
        message_bus_manager = None
        try_remote = False
        try:
            message_bus_manager = MessageBusManager(address=config["message_bus_address"], authkey=config['authkey']) 
            message_bus_manager.connect()
        except ConnectionRefusedError:
            print("local address failed")
            try:
                print(config["message_bus_address_remote"])
                message_bus_manager = MessageBusManager(address=config["message_bus_address_remote"], authkey=config['authkey']) 
                message_bus_manager.connect()
            except ConnectionRefusedError:
                print("remote address failed")
                raise 
        
        message_bus = message_bus_manager.mbus(None)
        return message_bus

    pass

class ProcessNames():
    bootstrap = "bootstrap"
    controller_interface = "controller_interface"
    lidar = "lidar"
    slam = "slam"
    planner = "planner"
    control_tower = "control_tower"
    robot_agent = "_robot"

class TopicNames():
    odometer = "odometer_updates"
    lidar = "lidar_updates"


