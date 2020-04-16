import os
import logging
import multiprocessing as mp
import time
import numpy as np
from messagebus import MessageBus, MessageBusProxy
from message import Message, Command
from config import Config
from messagebus_manager import MessageBusManager, ProcessNames

from lidar import LidarAgent
from controller_interface import ControllerInterface
from radio_controller import FutabaController

class goal_sources():
    radio_controller ="radio"
    control_tower="control_tower"
    plan="plan"


def futaba(msg, goals):
    config.log.info("director is running the futaba handler")

    if (len(msg.params) < 2 or len(msg.params[1]) < 16 ):
        config.log.warning("futaba handler parameter length mistmatch")
        return None

    new_controller = FutabaController(msg.params)
    config.log.info("radio controller (futaba) state is: " + ("ON" if new_controller.is_on else "OFF"))

    current_controller = goals[goal_sources.radio_controller]

    if (current_controller == None):
        # initialize first time
        current_controller = new_controller

    changes = current_controller.compare(new_controller)
    goals[goal_sources.radio_controller] = new_controller
    goals[goal_sources.radio_controller+".changed"] = changes

    radio = new_controller
    if radio.is_on:
        direction = radio.get_direction()
        speed = radio.get_speed()
        msg = Message(Message.move, params=[speed, direction, 101])
        return "serial_adapter", msg

    return None, None

def odometer(msg, goals):
    config.log.info("bootstrap is running the odometer handler")
    config.log.info("odometer= ({},{})".format(msg.params[0], msg.params[1]))
    return None, None

def do_nothing(msg, plans):
    return None, None

def invalid_command_id(msg, state):
    config.log.warning("bootstrap received an unknown command ({})".format(str(msg.cmd)))
    return None

command_handlers = {
    Command.futaba:futaba,
    Command.odometer:odometer,
}   

def make_plan(state, goals):
    radio = goals[goal_sources.radio_controller]
    radio_changes = goals[goal_sources.radio_controller+".changed"] 

    actions = []

    # if radio was swicthed on/off stop
    if radio_changes[0]:
            actions.append(("stop",) )

    if radio.is_on:
        # radio has higher priority 
        # calculate direction and speed
        direction = radio.get_direction()
        speed = radio.get_speed()

        actions.append( ("move", direction,speed) )
    else:
        actions.append( ("scheduled_action?",))

    return actions

if __name__ == '__main__':
    config = Config()
    mp.current_process().authkey = config['authkey']
    
    # goals represent what we want the robot to do
    # 
    goals = {goal_sources.radio_controller: None}
    state = {}

    MessageBusManager.register('mbus', MessageBus, MessageBusProxy)

    with MessageBusManager(address=config["message_bus_address"], authkey=config['authkey']) as message_bus_manager:
        
        message_bus = message_bus_manager.mbus(config)

        message_bus.send(ProcessNames.lidar, Message(command=Command.start))
        message_bus.send(ProcessNames.controller_interface, Message(command=Command.start))
        
        # start executor in local process - you can turn this off and start 
        # remotly using: "python executor.py"
        #lidar_worker = lidar.start(config)
        lidar_worker = LidarAgent.start_process(config)
        controller_interface_worker  = ControllerInterface.start_process(config)


        # wait until a message is received
        while True:
            try:
                msg = message_bus.receive(ProcessNames.bootstrap)
                config.log.info("bootstrap received: "  + msg.to_json())
                
                if msg.cmd == Command.shutdown: 
                    # shutdown all
                    msg=Message(command=Command.shutdown) 
                    for address in ProcessNames.__dict__.keys():
                        message_bus.send(address, msg)
                        
                    break

                # invoke a handler to update goals
                handler = command_handlers.get(msg.cmd, invalid_command_id)
                to_address, msg = handler(msg, goals)
                if (to_address is not None):
                    message_bus.send(to_address, msg)


            except (KeyboardInterrupt, SystemExit):
                config.log.warning("director received Ctrl+C ")
                break

            except Exception as e:
                config.log.error("director exception "  + str(e))

        #lidar_worker.terminate()
        controller_interface_worker.terminate()

        lidar_worker.join()
        controller_interface_worker.join()
        
    time.sleep(1)
    config.close()
    print("main process has terminated")
