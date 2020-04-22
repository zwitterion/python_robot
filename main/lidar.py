import os
import multiprocessing as mp
import numpy as np 

import threading
import time
import queue

from config import Config
from message import Message #, Command
from messagebus_manager import MessageBusManager, ProcessNames, TopicNames

from rplidar import RPLidar

# thread to process a plan
class LidarReader (threading.Thread):
    def __init__(self, config, mbus, event, lidar_data):
        threading.Thread.__init__(self)
        self.config = config
        self.message_bus = mbus
        self.event = event
        self.lidar_data = lidar_data
        self._running = True
        self.lidar_controller = RPLidar(config['lidar_port_name'])
        self.lidar_controller.stop()
        self.lidar_updates_topic_name = TopicNames.lidar

    def terminate(self): 
        self._running = False

    def publish_data(self):
        self.message_bus.send(self.lidar_updates_topic_name, Message(Message.lidar_data, self.lidar_data.tolist()))

        #self.config.log.info('lidar publish to (%s)' % self.lidar_updates_topic_name)

    def run(self):
            self.config.log.info('The lidar processor is creating topic: %s' % self.lidar_updates_topic_name)
            self.message_bus.create_topic(self.lidar_updates_topic_name)

            try: 
                self.config.log.info("waiting for lidar to start" )
                time.sleep(5)
                self.lidar_controller.clear_input()

                info = self.lidar_controller.get_info()
                self.config.log.info("lidar info: Model:{} Firmware:{}  Hardware:{}  SN:{}".format(
                    info['model'], info['firmware'], info['hardware'], info['serialnumber'] ))

                health = self.lidar_controller.get_health()
                self.config.log.info("lidar health: Status:{} Error Code:{}".format(
                    health[0], health[1] ))

                self.config.log.info("started reading loop...")
                # average N measurments per angle
                num_scans=0
                data = np.zeros((360, 2), dtype=int)
                for measurment in self.lidar_controller.iter_measurments():
                    if not self._running: 
                        self.lidar_controller.stop_motor()
                        print("*****************************************")
                        break

                    new_scan, quality, angle, distance = measurment
                    if (distance>0 and quality > 5):
                        theta = min(int(np.floor(angle)), 359)
                        data[theta,0] += distance
                        data[theta,1] += 1
                    
                    if new_scan: num_scans+=1
                    
                    if num_scans>10:
                        with np.errstate(divide='ignore',invalid='ignore'):
                            mean_distance = data[:,0]/data[:,1]
                        
                        # interpolate nan's
                        mask = np.isnan(mean_distance)
                        mean_distance[mask] = np.interp(np.flatnonzero(mask), np.flatnonzero(~mask), mean_distance[~mask])
                        np.copyto(self.lidar_data,mean_distance)
                        self.publish_data()
                        self.event.set()

                        # reset accumulators
                        data = np.zeros((360, 2), dtype=int)
                        num_scans = 0
                        
            
            except (KeyboardInterrupt, SystemExit):
                # this is not working... neeed to move inside rplidar code?
                self.lidar_controller.stop()
                self.lidar_controller.stop_motor()
                self.lidar_controller.disconnect()
                raise 

            finally:
                self.lidar_controller.stop_motor()
                self.lidar_controller.stop()
                self.lidar_controller.disconnect()


            return


class LidarAgent():
    def __init__(self, config):
        self.config = config
        self.name = "lidar"
        return
    
    @staticmethod 
    def _worker(config):
        lidar_agent = LidarAgent(config)
        lidar_agent.start()
        lidar_agent.run()
        return

    @staticmethod
    def start_process(config):
        main_worker = mp.Process(target=LidarAgent._worker, args=(config,))
        main_worker.start()
        return main_worker

    def start(self):
        self.config.log.info('The lidar processor is starting. ' + str(os.getpid()))
        self.message_bus = MessageBusManager.get_message_bus(self.config)
        self.event = threading.Event()
        self.lidar_data = np.zeros((360), dtype=float)
        self.lidar_reader = LidarReader(self.config, self.message_bus, self.event, self.lidar_data)
        self.lidar_reader.start()
        self.config.log.info('The lidar processor is ready')
        return

    def run(self):
 
        while True:
            try: 
                msg = self.message_bus.receive('lidar')
                self.config.log.info("lidar received:"  + str(msg.cmd))
                
                if (msg.cmd == Message.get_lidar_data):
                    # most common path to get data is via pubsub...
                    self.config.log.info("Got cmd: get_lidar_data. Waiting for lidar data")
                    if (self.event.wait(10)):
                        self.config.log.info("Lidar data is ready")
                        #print(lidar_data)
                        self.message_bus.send(msg.reply_to, Message(Message.lidar_data, self.lidar_data.tolist()))
                        continue

                if (msg.cmd == Message.shutdown):
                    self.config.log.info("lidar received shutdown command")
                    break
                
            except (KeyboardInterrupt, SystemExit):
                self.config.log.warning("lidar received Ctrl+C ")
                break                

            except Exception as e: 
                self.config.log.error("executor error " + str(e))
        
        self.config.log.info("shutting down lidar process")
        self.lidar_reader.terminate()
        self.lidar_reader.join()

        self.config.log.info("lidar terminated")
    
        return


if __name__ == '__main__':
    config = Config()
    #main_worker = start(config)
    #main_worker.join()
    #print("lidar process completed")

    lidar_agent = LidarAgent(config)
    lidar_agent.start()
    lidar_agent.run()
