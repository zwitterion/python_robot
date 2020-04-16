import os
import sys
import numpy as np
import multiprocessing as mp
from timeit import default_timer as timer

import networkx as nx 

from messagebus_manager import MessageBusManager
from config import Config
from message import Message, Command

class Planner():
    def __init__(self, config):
        self.config = config
        self.name = "planner"
        self.handlers = { 
            Message.get_map: self.process_get_map_request,
            Message.map_update: self.process_map_update_request,
            Message.set_target_position: self.process_set_target_position_request

            }
        self.map = Map(self.config["map_grid_size"])
        self.map_updates_topic_name = "map_updates"
        self.target_position = [-1,-1]
        return

    def process_get_map_request(self, msg):
        self.send_map(msg.reply_to)

    def process_map_update_request(self, msg):
        self.config.log.info('planner has received a new map')
        data = np.array(msg.params['grid'])
        self.map.grid = data
        self.map.robot_position = msg.params['robot_position']
        assert self.map.robot_position[0]<data.shape[0], "invalid X position"
        assert self.map.robot_position[1]<data.shape[1], "invalid Y position"
        
        if self.map.robot_position[0] > (data.shape[0]-1.  + 0.5):
            self.map.robot_position[0] = data.shape[0]-1.0
        if self.map.robot_position[1] > (data.shape[1]-1.  + 0.5):
            self.map.robot_position[1] = data.shape[1]-1.0

        self.map.target_position = self.target_position
        self.map.update_path()

        self.publish_map()
        return

    def process_set_target_position_request(self, msg):
        
        self.target_position = msg.params
        self.config.log.warning("planner got new target ({})".format(str(self.target_position)))

        self.map.target_position = self.target_position
        self.map.update_path()
        self.publish_map()

        return

    def handle_invalid_command_id(self, msg):
        self.config.log.warning("planner received an unknown command ({})".format(str(msg.cmd)))
        return


    def publish_map(self):
        self.send_map(self.map_updates_topic_name)

    def send_map(self, address):
        self.message_bus.send(address, Message(Message.map_update,  self.map.tolist()))
        self.config.log.info('planner sent map update to (%s)' % address)
        return
        

    def start(self):

        self.config.log.info('The planner is starting ' + str(os.getpid()))
        try:
            self.message_bus = MessageBusManager.get_message_bus(self.config)
        except ConnectionRefusedError as e:
            self.config.log.error('The message bus is not accessible')
            print(e)
            raise e

        self.message_bus.create_topic(self.map_updates_topic_name)

        while True:
            try:
                msg = self.message_bus.receive(self.name)
                handler = self.handlers.get(msg.cmd, self.handle_invalid_command_id)
                handler(msg)
                

            except (KeyboardInterrupt, SystemExit):
                self.config.log.info("control tower got Ctrl+C" )
                return # exit process

            except (EOFError, BrokenPipeError):
                self.config.log.warning("control tower got broken pipe exception" )
                return # exit process


            except Exception as e:
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                self.config.log.error("Planner Got Exception: type:{} line: {}".format(exc_type, exc_tb.tb_lineno))

                print (os.path.split(exc_tb.tb_frame))


        return

    def close(self):
        self.config.log.info("planner is closing connections to message bus")
        self.message_bus._close()

    def __del__(self):
        self.close()

class Map():
    def __init__(self, grid_size):
        self.config = config
        self.grid = np.ones(grid_size)
        self.robot_position = [-1,-1,0]
        self.target_position = [-1,-1]
        self.path = []
        
        # create grid graph
        self.graph = nx.grid_graph(dim=list(grid_size) )
        #grid_2d_graph

        # connect diagonals
        # https://stackoverflow.com/questions/55772715/how-to-create-8-cell-adjacency-map-for-a-diagonal-enabled-a-algorithm-with-the
        
        self.graph.add_edges_from([
            ((x, y), (x+1, y+1))
            for x in range(grid_size[0]-1)
            for y in range(grid_size[1]-1)
        ] + [
            ((x+1, y), (x, y+1))
            for x in range(grid_size[0]-1)
            for y in range(grid_size[1]-1)
        ], weight=0.5)

        for n in self.graph.nodes:
            self.graph.nodes[n]["weight"] = 0.5

        return

    def convert_to_weight(self,v):
        if v > 200:
            return 0

        return 255-v

    def update_path(self):
        # set graph weigths 
        for n in self.graph.nodes:
            self.graph.nodes[n]["weight"] =  self.convert_to_weight(self.grid[n[1], n[0]] )

        # these are the weigths that matter
        # set the edge weight to the mean of the two nodes weights. 
        for e in self.graph.edges:
            self.graph.edges[e]["weight"] =  (self.graph.nodes[e[0]]["weight"] + self.graph.nodes[e[1]]["weight"])/2.0

        # get path
        self.path = self.get_path()

        #print("PATH")
        for n in self.path:
            print( self.graph.nodes[n] )

        return

    def get_path(self):
        def cost_heuristic(a,b):
            (x1,y1) = a 
            (x2,y2) = b 
            return ((x1-x2)**2 + (y1-y2)**2)**0.5 
        
        start = tuple(np.round(self.robot_position[0:2]))
        goal = tuple(self.target_position)


        if (goal[0]!=-1):
            path = nx.astar_path( self.graph, start,goal, cost_heuristic, weight="weight")
        else:
            path = []

        # debug

        return path

    def tolist(self):

        data = { 'grid': self.grid.tolist(),
                 'robot_position': self.robot_position,
                 'target_position': self.target_position,
                 'path': self.path }
        
        return data

    def __str__(self):
        return "<Map %s %s>" % (self.grid.shape)

    def __repr__(self):
            return self.__str__()

class InvalidReplyException(Exception):
    pass

if __name__ == '__main__':
    config = Config()
    planner = Planner(config)
    planner.start()
    
    
    
