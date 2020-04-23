import os
import sys
import multiprocessing as mp
import traceback
from collections import namedtuple
from timeit import default_timer as timer
import numpy as np
import math

from messagebus_manager import MessageBusManager, ProcessNames, TopicNames
from config import Config
from message import Message#, Command


class RobotStates():
    is_idle = 0
    is_rotating = 1
    is_moving = 2
    is_waiting_for_map_update = 3
    state_strings = {is_idle: "idle",
                     is_rotating: "rotate",
                     is_moving: "move",
                     is_waiting_for_map_update: "waiting_for_map",
                    }
    @staticmethod
    def to_string(s):
        return RobotStates.state_strings[s]

class Odometer():
    def __init__(self, l,r):
        self.left_wheel = l
        self.right_wheel = r

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
        self.odometer = Odometer(0,0)
        # set before starting to move to track coordinates of next stop/major change in direction
        self.next_stop = [-1, -1]

        # tracks last value of robot orientation returned by SLAM. It is used to determine when orienation stops changing
        # after a rotation, it might be unstable for few milliseconds 
        self.last_robot_orientation = 999

        self.steps_per_turn = 98.0
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

        # stop any robot movement 
        self.stop()

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
                exc_type, _, exc_tb = sys.exc_info()
                #fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                self.config.log.error("{} got exception: type:{} line: {}".format(self.name, exc_type, exc_tb.tb_lineno))
                traceback.print_exc()

        return

    def on_map_update(self, msg):
        
        #self.config.log.info("{} got map udpate".format(self.name) )        
        self.grid = msg.params['grid']
        self.robot_position = msg.params['robot_position']
        self.target_position = msg.params['target_position']
        self.path = msg.params['path']

        if self.state == RobotStates.is_idle and len(self.path) <1:

            return

        
        tgt_theta =  self.get_orientation_to_target( self.robot_position[0:2], self.next_stop)

        # self.target_theta
        robot_orientation = self.get_current_orientation()        
        #orientation_error = (robot_orientation - tgt_theta) if tgt_theta is not None else 0
        orientation_error = self.rotation_angle(tgt_theta, robot_orientation)

        self.config.log.info("state: %s c_xy(%6.2f,%6.2f) t_xy(%6.2f,%6.2f) odo: (L:%3d, R:%3d) tgt theta: %6.2f theta: %6.2f error: %6.2f" % 
                (RobotStates.to_string(self.state), 
                self.robot_position[0], 
                self.robot_position[1], 
                self.path[0][0],
                self.path[0][1],
                self.odometer.left_wheel,
                self.odometer.right_wheel,
                tgt_theta, 
                robot_orientation ,
                orientation_error))

        if self.state == RobotStates.is_rotating:
            if self.active_path[-1] != self.path[-1]: 
                print ("STOP ROTATION TGT HAS CHANGED")
                print("path:", self.path,"\n", "active:", self.active_path[-len(self.path):] )
                self.stop()

        if self.state == RobotStates.is_moving:
            if (len(self.path)>=2):
                
                euclidean_distance = self.euclidean_distance(self.robot_position[0:2], self.path[1])
                active_path = self.active_path[-len(self.path):][0:2]
                if len(self.path)>=2 and active_path[0:2] != self.path[0:2] and self.get_cell_rotation(active_path[0], active_path[1]) in [-90,90,180,-180]:
                    # if robot is busy and the path has changed then something is wrong - tell robot to cancel 
                    # then it will continue with the new path
                    # ignore if was moving in a diagonal
                    print ("STOP !!!!!!!!!!!*****************************")
                    print("path:", self.path,"\n", "active:", self.active_path[-len(self.path):], "\neucl dist:", euclidean_distance, "\nobot:", self.robot_position )
                    self.stop()
                    return

            # update move parameters
                # get max travel distance (without requiring a rotation)
            distance = self.get_max_distance(self.path)
            if (distance < len(self.path)):
                self.next_stop = self.path[distance]
                euclidean_distance = self.euclidean_distance(self.robot_position[0:2], self.next_stop)
                heading = np.clip(-orientation_error, -5, 5)
                speed = 0.6 if distance >= 2 else 0.3
                self.move(euclidean_distance, speed=speed, heading=heading)
            else:
                print("Why is distance wrong?", distance, self.path)

        if self.state == RobotStates.is_waiting_for_map_update:
            if self.is_orientation_stable(robot_orientation): 
                # reset to idle
                self.state = RobotStates.is_idle
            else:
                print("***unstable")

        return

    def on_odometer_update(self, msg):
        if (len(msg.params) >=2):
            self.odometer.left_wheel = msg.params[0]
            self.odometer.right_wheel = msg.params[1]
        else:
            self.config.log.error("robot: {} received bad odometer data".format(self.name))    
        return

    def on_command_completed(self, msg):
        self.config.log.info("robot: {} completed an action".format(self.name))
        # wait for new map with robot position updated
        # reset state to idle once a new map arrives
        if (len(msg.params) >=4):
            self.odometer.left_wheel = msg.params[2]
            self.odometer.right_wheel = msg.params[3]

        # wait for next map/position update (and ofr the orientation become stable)
        if self.state != RobotStates.is_idle:
            self.state = RobotStates.is_waiting_for_map_update

        return

    def on_invalid_command_id(self, msg):
            self.config.log.warning("robot: {} received an unknown command ({})".format(self.name, str(msg.cmd)))
            return

    def is_orientation_stable(self, new_orientation):
        """
        Returns true when orientation stops changing for less tha 1.0
        """
        stable = abs(self.last_robot_orientation - new_orientation) < 1.0
        self.last_robot_orientation = new_orientation
        return stable    


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
            #error =  -error
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


        # get max travel distance (without requiring a rotation)
        distance = self.get_max_distance(self.path)
        
        # next_orientation 
        #theta = self.get_cell_rotation(self.path[0], self.path[1])

        # angle of vector from current robot position to next target position
        # might be slightly difference than starting from self.path[0]
        theta = self.get_orientation_to_target( self.robot_position[0:2], self.path[distance])

        # where is robot pointing at
        robot_orientation = self.get_current_orientation()

        self.target_theta = theta

        # rotate the robot so it points to the target
        if not self.rotate(theta, robot_orientation): 
            
            # if we don't need to rotate then move
            self.next_stop = self.path[distance]
            assert distance>=0, "distance should be >=0"
            
            if (distance>0):
                next_stop = self.path[distance]
                euclidean_distance = self.euclidean_distance(self.robot_position[0:2], next_stop)
                self.move(euclidean_distance)
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

    def get_orientation_to_target(self, a, b):
        """
            returns tangent a-b
            =IF(F2<>0,ATAN(G2/F2), SIGN(G2)*PI()/2) + IF(F2>=0,0, IF(G2>=0,PI(), -PI()))
            normalize so it returns values between [0, 180] for CCW rotations or (-0, -180) for CW rotations
        """
        (x1, y1) = a
        (x2,y2) = b
        alpha = 0

        dx = x2-x1
        dy = y2-y1
        alpha = (math.atan(dy/dx) if (dx!=0) else np.sign(dy) * (np.pi/2.0)) + (0.0 if (dx>=0) else np.pi if dy>=0 else -np.pi)

        return (alpha * 360.) / (2.*np.pi)

    def rotation_angle(self, alpha, robot_orientation):
        
        CW = 1
        CCW = -1
        heading = 0

        if (alpha>robot_orientation):
            d1 = alpha - robot_orientation
            d2 = 360-alpha+robot_orientation
            delta = min(d1,d2)
            heading = delta * (CCW if (d1<d2) else CW)
        else:
            d1 = robot_orientation - alpha 
            d2 = 360-robot_orientation+alpha
            delta = min(d1,d2)
            heading = delta * (CW if (d1<d2) else CCW)
        return heading


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

        rotate_threshold_in_steps = 2
        # rotating for small distances does not work quite well with this robot???
        # and can start oscillating...
        # todo: upgrade arduino code to use microsteps for higher resolution..
        if (steps>rotate_threshold_in_steps):
            self.config.log.info("Robot is starting to rotate to: {}   v:{} h:{} steps:{}".format(alpha, speed, heading, steps))
            msg = Message(Message.move, params=[speed, heading, steps])
            msg.reply_to = self.name
            self.message_bus.send(ProcessNames.controller_interface , msg)
            rotate_started = True
            self.state = RobotStates.is_rotating

        return rotate_started

   

    def move(self, distance, speed=0.6, heading=0):
        
        #next_stop = self.path[distance]
        #distance = self.euclidean_distance(self.robot_position[0:2], next_stop)
        steps = int(distance * self.steps_per_cell)

        self.config.log.info("Robot is starting to move: {:6.3f} unit(s) v:{} h:{} steps:{}".format(distance, speed, heading, steps))
        msg = Message(Message.move, params=[speed, heading, steps])
        msg.reply_to = self.name
        self.message_bus.send(ProcessNames.controller_interface, msg)
        self.state = RobotStates.is_moving
        return

    def send_move_error(self, path_error):
        #self.config.log.info("move error: {:6.3f} ".format(path_error))
        msg = Message(Message.move_error, params=[path_error*0.1])
        msg.reply_to = self.name
        self.message_bus.send(ProcessNames.controller_interface, msg)
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

    def euclidean_distance(self, p0, p1):
        return np.linalg.norm(np.array(p1)-np.array(p0))
        #return math.sqrt(math.pow(p0[0]-p1[0], 2) + math.pow(p0[1]-p1[1], 2))


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
    
    
