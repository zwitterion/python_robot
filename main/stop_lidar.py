#!/usr/bin/env python3
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

PORT_NAME = '/dev/ttyUSB0'


def run():
    lidar = RPLidar(PORT_NAME)
    
    lidar.stop()

    info = lidar.get_info()
    print(info)
 
    health = lidar.get_health()
    print(health)
    print(lidar)    
    lidar.stop()
    lidar.stop_motor()
   
   
    lidar.disconnect()
    print(dir(lidar))

if __name__ == '__main__':
    run()
