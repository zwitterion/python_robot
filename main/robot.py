import os
import sys
import multiprocessing as mp
import traceback
from messagebus_manager import MessageBusManager, ProcessNames, TopicNames
from config import Config
from message import Message, Command
from timeit import default_timer as timer
import numpy as np

class RobotStates():
    is_idle = 0
    is_rotating = 1
    is_moving = 2

class Robot():
    def __init__(self, config):
        self.config = config
        self.name = "_robot"
        self.planner_updates_topic_name = "map_updates"
        self.path = []
        self.active_path = [] # path when robot started executing
        self.target_theta = -1 # target orientation
        self.robot_position = [-1,-1,-1]
        self.target_position = self.robot_position
        self.state = RobotStates.is_idle
        self.wait_for_map = False
        self.handlers = []
        self.message_bus = None

        self.steps_per_turn = 100.0
        self.steps_per_cell = 15.

        return

    def __del__(self):
        self.close()

    def start(self):

        self.config.log.info('The robot is starting ' + str(os.getpid()))
        try:
            self.message_bus = MessageBusManager.get_message_bus(self.config)
        except ConnectionRefusedError as e:
            self.config.log.error('The message bus is not accessible')
            print(e)
            raise e

        self.message_bus.subscribe(topic = self.planner_updates_topic_name, subscriber=self.name)
        self.message_bus.subscribe(topic = TopicNames.odometer, subscriber=self.name)

        self.config.log.info("robot subscribed to %s" % self.planner_updates_topic_name)

        self.handlers = { 
            Message.map_update: self.on_map_update,
            Message.odometer: self.on_odometer_update,
            Message.command_completed: self.on_command_completed
        }

        # remove any old messages
        self.message_bus.clear(self.name)

        self.config.log.info('The robot base start has completed')

        return


    def run(self):
        
        self.start()

        while True:
            try:
                msg = self.message_bus.receive(self.name)
                handler = self.handlers.get(msg.cmd, self.on_invalid_command_id)
                handler(msg)
            
                # start move or wait until on_command_completed_fires to do next action
                if (self.state == RobotStates.is_idle):
                    self.execute_next_action()

            except (KeyboardInterrupt, SystemExit):
                self.config.log.info("{} got Ctrl+C".format(self.name) )
                return # exit process

            except (EOFError, BrokenPipeError):
                self.config.log.warning("{} got broken pipe exception".format(self.name) )
                return # exit process


            except Exception as e:
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                self.config.log.error("{} got exception: type:{} line: {}".format(self.name, exc_type, exc_tb.tb_lineno))
                traceback.print_exc()

        return

    def on_map_update(self, msg):
        
        #self.config.log.info("{} got map udpate".format(self.name) )        
        self.grid = msg.params['grid']
        self.robot_position = msg.params['robot_position']
        self.target_position = msg.params['target_position']
        self.path = msg.params['path']


        #path_error = self.get_path_error()
        #print("error", path_error)
        robot_orientation = self.get_current_orientation()
        
        self.config.log.info("x: %6.2f y: %6.2f  target theta: %6.2f current theta: %6.2f" % (self.robot_position[0], self.robot_position[1], self.target_theta, robot_orientation))

        # if robot is busy and the path has changed then something is wrong - tell robot to cancel 
        # then it will continue with the new path
        if self.state == RobotStates.is_moving:
            if self.path != self.active_path[len(self.active_path)-len(self.path):]:
                print ("STOP !!!!!!!!!!!*****************************")
                print(self.path,"\n", self.active_path[len(self.active_path)-len(self.path):] )
                self.stop()

        if self.wait_for_map:
            # reset busy
            self.state = RobotStates.is_idle
            self.wait_for_map = False

        return

    def on_odometer_update(self, msg):
        #print("ODOMETER!!!!!!!!!", msg)
        return

    def on_command_completed(self, msg):
        self.config.log.info("robot: {} completed an action".format(self.name))
        # wait for new map with robot position updated
        # reset state to idle once a new map arrives
        self.wait_for_map = True

        return

    def on_invalid_command_id(self, msg):
            self.config.log.warning("robot: {} received an unknown command ({})".format(self.name, str(msg.cmd)))
            return

    def get_current_orientation(self):
        robot_orientation = self.robot_position[2]
        if robot_orientation > 0:
            robot_orientation = robot_orientation % 360
        else:
            robot_orientation = -1.0* (-robot_orientation) % 360

        # transform to -180, +180
        if robot_orientation > 180: robot_orientation = robot_orientation-360
        if robot_orientation < -180: robot_orientation = 360-robot_orientation

        return robot_orientation
        

    def get_path_error(self):
        # self.path, self.robot_position
        if len(self.path) < 2:
            return 0 # ??

        p0 = self.path[0]
        p1 = self.path[1]
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        if dx != 0:
            a = (dy/dx) 
            b = -a*p0[0] + p0[1]
            error = (a * self.robot_position[0] + b) - self.robot_position[1]
        elif dy != 0:
            a = (dx/dy) 
            b = -a*p0[1] + p0[0]
            error = (a * self.robot_position[1] + b) - self.robot_position[0]
        else:
            #???
            error = 0

        return error


    def execute_next_action(self):
        if (len(self.path) <= 1):
            # nothing to do
            return

        assert self.state == RobotStates.is_idle, "cannot execute an action while another one is pending"
        
        # keep a copy of the path so we can detect changes while robot is busy executig a command
        self.active_path = self.path

        
        # next_orientation 
        theta = self.get_cell_rotation(self.path[0], self.path[1])

        robot_orientation = self.get_current_orientation()

        self.target_theta = theta

        if not self.rotate(theta, robot_orientation):
            # if we don't need to rotate then move
            distance = self.get_max_distance(self.path)
            assert distance>=0, "distance should be >=0"
            
            if (distance>0):
                self.move(distance)
            else:
                self.state = RobotStates.is_idle
        return

    def get_cell_rotation(self, a, b):
        """
            returns 0,45,90,...180
        """
        (x1, y1) = a
        (x2,y2) = b
        alpha = 0

        # y is inverted !@#!#!
        if (x1 == x2): 
            alpha = -90 if (y2<y1) else 90
        elif (y1==y2):
            alpha = 0 if (x2>x1) else 180
        elif (x1 == x2-1 and y1 == y2+1):
            alpha = -45
        elif (x1 == x2+1 and y1 == y2-1):
            alpha = +135
        elif (x1 == x2+1 and y1 == y2+1):
            alpha = -135
        elif (x1 == x2-1 and y1 == y2-1):
            alpha = +45

        #print (x1,y1,x2,y2, alpha)

        return alpha

    def rotate(self, alpha, robot_orientation):

        speed = 0.3
        delta = abs(robot_orientation-alpha)
        CW = 90
        CCW = -90
        rotate_started = False

        if (alpha>robot_orientation):
            d1 = alpha - robot_orientation
            d2 = 360-alpha+robot_orientation
            delta = min(d1,d2)
            heading = CCW if (d1<d2) else CW
        else:
            d1 = robot_orientation - alpha 
            d2 = 360-robot_orientation+alpha
            delta = min(d1,d2)
            heading = CW if (d1<d2) else CCW

        steps = int(float(delta) * self.steps_per_turn/360.0)
            
        #print("ROTATING:", speed, heading, steps )
        assert steps >= 0, "invalid # of steps"

        if (steps>0):
            self.config.log.info("Robot is starting to rotate to: {}".format(alpha))
            msg = Message(Message.move, params=[speed, heading, steps])
            msg.reply_to = self.name
            self.message_bus.send(ProcessNames.controller_interface , msg)
            rotate_started = True
            self.state = RobotStates.is_rotating

        return rotate_started

   

    def move(self, distance, speed=0.3):
        self.config.log.info("Robot is starting to move: {} unit(s)".format(distance))
        heading = 0
        steps = int(distance * self.steps_per_cell)
        msg = Message(Message.move, params=[speed, heading, steps])
        msg.reply_to = self.name
        self.message_bus.send(ProcessNames.controller_interface, msg)
        self.state = RobotStates.is_moving
        return

    def stop(self):
        self.config.log.info("Robot is starting to stop")
        msg = Message(Message.stop)
        msg.reply_to = self.name
        self.message_bus.send(ProcessNames.controller_interface, msg)
        return

    def get_max_distance(self, path):
        """
        Returns max number of cells robot can traverse with the same orientation of first two cells
        """        
        if (len(path)<2):
            return len(path)
        r = self.get_cell_rotation(path[0], path[1])
        i=1
        while (i < len(path)-1 ) and (self.get_cell_rotation(path[i], path[i+1]) == r):
            i+=1
        return i

    def log(self, text):
        self.config.log.info(self.name + ": " + text)

    def close(self):
        if self.message_bus != None:
            self.config.log.info("Robot is closing connections to message bus")
            self.message_bus._close()

        
class RobotState():
    def __init__(self, left, right):
        self.left_wheel = left
        self.right_wheel = right
        return
    
    def __repr__(self):
            return "<RobotState left_wheel:%s right_wheel:%s>" % (self.left_wheel, self.right_wheel)

class InvalidReplyException(Exception):
    pass

if __name__ == '__main__':
    config = Config()
    robot = Robot(config)
    robot.run()
    
    
