#!/usr/bin/env python3

'''
Based on BreezySLAM sample code:
    rpslam.py : BreezySLAM Python with SLAMTECH RP A1 Lidar
    Copyright (C) 2018 Simon D. Levy

https://simondlevy.academic.wlu.edu/files/students/BreezySLAM_SurajBajracharya.pdf

'''
import os
import numpy as np
from scipy import signal
from PIL import Image
import multiprocessing as mp
from messagebus_manager import MessageBusManager, ProcessNames, TopicNames
from config import Config
from message import Message
from timeit import default_timer as timer

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from breezyslam.vehicles import WheeledVehicle

from rplidar import RPLidar as Lidar
from roboviz import MapVisualizer


MAP_SIZE_PIXELS         = 400    # 200
MAP_SIZE_METERS         = 10     # 10


class ArloRobot(WheeledVehicle):
    # https://www.parallax.com/product/28966
    def __init__(self):
        # d=155mm 
        WheeledVehicle.__init__(self, wheelRadiusMillimeters=77.5, halfAxleLengthMillimeters=190.5)
        previous_left_wheel = 0
        previous_right_wheel = 0

    def extractOdometry(self, timestamp, leftWheelOdometry, rightWheelOdometry):

        return seconds, alphaL, alphaR

class SLAM():
    def __init__(self, config):
        self.config = config
        self.name = "slam"
        self.slam = None
        self.viz = None
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.grid_size = config['map_grid_size']
        self.subscribers = [ProcessNames.planner]   # todo change to new publish/sub
        self.lidar_updates_topic_name = TopicNames.lidar

        return

    def start(self):
        self.config.log.info('SLAM is starting ' + str(os.getpid()))
        try:
            self.message_bus = MessageBusManager.get_message_bus(self.config)
        except ConnectionRefusedError as e:
            self.config.log.error('The message bus is not accessible')
            print(e)
            raise e
        
        # clear my queue
        self.message_bus.clear(self.name)
        self.config.log.info('SLAM initialization completed')

        return

    def update_map(self, msg):

        self.config.log.info("starting slam update")

        distances = msg.params
        angles    = [360-a for a in range(360)]

        # Update SLAM with current Lidar scan and scan angles
        self.slam.update(distances, scan_angles_degrees=angles)
   
        # Get current robot position
        x, y, theta = self.slam.getpos()

        # Get current map bytes as grayscale
        self.slam.getmap(self.mapbytes)

        # publish map to subscribers
        self.publish_map()

        # Display map and robot pose, exiting gracefully if user closes it
        if not self.viz.display(x/1000., y/1000., theta, self.mapbytes):
            exit(0)
        
        return

    def publish_map(self):
        for subscriber in self.subscribers:
            self.send_map(subscriber)


    def process_get_map_command(self, msg):
        self.send_map(msg.reply_to)
        return

    def process_reset_map_command(self, msg):
        #self.mapbytes = bytes(len(self.mapbytes))
        # this does not work yet!!!
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.slam.setmap(self.mapbytes)
        return

    def send_map(self, send_to_address):

        # Get current robot position
        robot_x, robot_y, robot_theta = self.slam.getpos()

        # Get current map bytes as grayscale
        self.slam.getmap(self.mapbytes)
        base_grid = np.array(self.mapbytes).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
        
        grid = self.resize_array(base_grid, self.grid_size, resample=Image.NEAREST)

        if self.config["slam.set_boundaries"]:
            grid = self.set_boundaries(grid.copy(), 1500)

        # scale robot position to grid coordinates
        robot_x_grid = (robot_x/(1000*MAP_SIZE_METERS))*( self.grid_size[0] - 1)
        robot_y_grid = (robot_y/(1000*MAP_SIZE_METERS))*( self.grid_size[1] - 1)
        
        #if robot_x_grid > self.grid_size[0]

        #print("robotx", robot_x, robot_x_grid)

        self.config.log.info("slam sending map to ({})".format(send_to_address))
        self.message_bus.send(send_to_address, Message(Message.map_update, params={
                        'robot_position': [robot_x_grid, robot_y_grid, robot_theta], 
                        'grid': grid.tolist()}))

        return 


    def resize_array(self, a, size, resample=Image.NEAREST):
        img = Image.fromarray(a) #, 'L')
        img2 = img.resize(size, resample=resample)
        return np.asarray(img2)

    def set_boundaries(self, grid, threshold):
        """
         adds a boundary region around obstacles (increases the size of the obstacles)
        """
        scharr = np.array([[ -3-3j, 0-10j,  +3 -3j],
                        [-10+0j, 0+ 0j, +10 +0j],
                        [ -3+3j, 0+10j,  +3 +3j]]) # Gx + j*Gy

        dark = np.where(grid<200)
        grid[dark] = 0

        grad = signal.convolve2d(grid, scharr, boundary='symm', mode='same')
        mask = np.absolute(grad)
        mask[np.where(mask<threshold)] = 0
        mask[np.where(mask>=threshold)] = 128 # 255
        new_grid = grid-mask
        new_grid[np.where(new_grid < 0)]=0
        
        return new_grid

    def handle_invalid_command_id(self, msg):
        self.config.log.warning("slam received an unknown command ({})".format(str(msg.cmd)))
        return 

    def run(self):

        # Create an RMHC SLAM object with a laser model and optional robot model
        self.slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

        # Set up a SLAM display
        self.viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

        command_handlers = {
            Message.lidar_data:  self.update_map,
            Message.get_map:  self.process_get_map_command,
            Message.reset_map:  self.process_reset_map_command,
            }


        self.message_bus.subscribe(topic = self.lidar_updates_topic_name, subscriber=self.name)
        self.config.log.info("slam subscribed to %s" % self.lidar_updates_topic_name)

        while True:

            msg = self.message_bus.receive(self.name)

            handler = command_handlers.get(msg.cmd, self.handle_invalid_command_id)
            handler(msg)
                        
        return        

    def log(self, text):
        self.config.log.info(self.name + ": " + text)

    def close(self):
        self.config.log.info("slam is closing connections to message bus")
        self.message_bus._close()

    def __del__(self):
        self.close()

class SlamState():
    def __init__(self, left, right):
        self.left_wheel = left
        self.right_wheel = right
        return
    
    def __repr__(self):
            return "<slam...>"

class InvalidReplyException(Exception):
    pass

if __name__ == '__main__':
    config = Config()
    slam = SLAM(config)
    slam.start()
    slam.run()
    
    
    
