import os
import sys
import multiprocessing as mp
import threading
import time
import queue
import numpy as np 

from messagebus_manager import MessageBusManager, ProcessNames, TopicNames
from config import Config
from message import Message, Command
#from pySerialTransfer 
import pySerialTransfer as txfer

from multiprocessing import Array

exitFlag = 0


class ExternalMessage():

   def __init__(self, buffer):
         self.command = buffer[0]
         self.buffer = buffer
         self.commands = {
            Message.status: self.status,
            Message.futaba: self.futaba,
            Message.odometer: self.odometer,
            Message.move: self.move,
            Message.command_completed: self.command_completed
         }

         # handler to fetch proper commands
         self.params = self.commands[self.command]
         
         return 

   def move(self):
      x = self.buffer[1] + (self.buffer[2] << 8)
      y = self.buffer[3] + (self.buffer[4] << 8)
      return (x, y)

   def odometer(self):
      left = self.buffer[1] + (self.buffer[2] << 8)
      right = self.buffer[3] + (self.buffer[4] << 8)
      return (left, right)

   def futaba(self):
      status = self.buffer[1] + (self.buffer[2] << 8)
      channels  = [self.buffer[i] + (self.buffer[i+1] << 8) for i in range(0+3,32+3,2)]
      return (status, channels)

   def status(self):
      len = self.buffer[1:].index(0)
      text = "".join(chr(c) for c in self.buffer[1:len+1])
      return (text,)

   def command_completed(self):
      reply_to = self.buffer[1] 
      status = self.buffer[2] 
      left_counter = self.buffer[3] + (self.buffer[4] << 8)
      right_counter = self.buffer[5] + (self.buffer[6] << 8)
      return (reply_to, status, left_counter, right_counter)



# thread to process incoming data
class ProcessIncominData (threading.Thread):
    def __init__(self, config, link, mbus, reply_to_table):
        threading.Thread.__init__(self)
        self.config = config
        self.link = link
        self.message_bus = mbus
        self.reply_to_table = reply_to_table
        self.daemon = True
        self.name = ProcessNames.controller_interface

    def callback(self, buffer, bytes_read):

        external_msg = ExternalMessage(buffer)

        if (external_msg.command == Message.futaba):
            msg = Message(command=external_msg.command, params=external_msg.params())
            self.message_bus.send(ProcessNames.bootstrap, msg )
        elif external_msg.command == Message.odometer:
            # publish odometer data
            msg = Message(command=external_msg.command, params=external_msg.params())
            self.message_bus.send(TopicNames.odometer, msg )
        elif (external_msg.command == Message.command_completed):
            self.config.log.info("serial_adapter received command_completed" )
            reply_to = self.reply_to_table[external_msg.params()[0]]
            msg = Message(command=external_msg.command, params=external_msg.params())
            self.config.log.info("serial_adapter sent command_completed to:{}".format(reply_to) )
            print("MSG", msg.to_json())
            self.message_bus.send(reply_to, msg )
        elif (external_msg.command == 1):
            print("*****STATUS INFO****", external_msg.params())
            #self.config.log.info("serial_adapter recived status=" )
        else:
            print("*****UNKNOWN CMD****", external_msg.params())
        return

    def run(self):
            self.config.log.info("{} starting input data processing thread ".format(self.name) )


            self.message_bus.create_topic(TopicNames.odometer)
            self.config.log.info("{0} created topic: {1} ".format(self.name, TopicNames.odometer) )

            try:
                # never returns
                self.link.read_data(self.callback)
                
            except (KeyboardInterrupt, SystemExit):
                self.config.log.info("serial_adapter Ctrl+C" )
                return # exit process

            except (EOFError, BrokenPipeError):
                self.config.log.warning("serial_adapter: exiting ProcessIncominData" )
                return # exit process

            except Exception as e:
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                self.config.log.error("serial_adapter: ProcessIncominData: type:{} line: {}".format(exc_type, exc_tb.tb_lineno))
       
            return

# thread to process outgoing data
class ProcessOutgoingData (threading.Thread):
   def __init__(self, config, link, send_items, reply_to_table):
      threading.Thread.__init__(self)
      self.config = config
      self.link = link
      self.send_items = send_items
      self.reply_to_table = reply_to_table
      self.daemon = True

   def get_reply_to_id(self,reply_to):
        if (reply_to not in self.reply_to_table):
           self.reply_to_table.append(reply_to)
        reply_to_int = self.reply_to_table.index(reply_to)
        return reply_to_int

   def run(self):
        self.config.log.info("ProcessOutgoingData started output data processing thread " )
        i=0
        while True: 
            try:
                item = self.send_items.get()
                # send it
                cmd = item.cmd
                params = item.params
                # assign short id to reply to address to keep payload small 
                reply_to_int = self.get_reply_to_id(item.reply_to)

                if cmd == Message.move:
                    speed = int(params[0]*1000)
                    if (speed<-1100 or speed > 1100):
                        self.config.log.error("ProcessOutgoingData error speed is invalid: " + str(speed))
                    speed = np.clip(speed, -1000, +1000)
                    speed += 1000

                    direction = int(params[1]*100)
                    if (direction<-9100 or direction > 9100):
                        self.config.log.error("ProcessOutgoingData error direction is invalid: " + str(direction))
                    
                    direction = np.clip(direction, -9000, 9000)
                    direction +=9000

                    distance = int(params[2])

                    self.link.txBuff[0] = 100
                    self.link.txBuff[1] = (speed & 0x00FF)
                    self.link.txBuff[2] = (speed >> 8)
                    self.link.txBuff[3] = (direction & 0x00FF)
                    self.link.txBuff[4] = (direction >>8)
                    self.link.txBuff[5] = (distance & 0x00FF)
                    self.link.txBuff[6] = (distance >>8)
                    self.link.txBuff[7] = reply_to_int

                    error = False
                    for i in range(8):
                        if self.link.txBuff[i] > 255:
                            self.config.log.error("ProcessOutgoingData: value must be a byte " + str(i))
                            error = True
                    
                    if (not error):
                        if self.link.send(8):
                            self.config.log.info("ProcessOutgoingData sent via serial: Comand:{} speed:{} dir:{} distance:{} reply:{}".format(cmd, speed, direction, distance, reply_to_int) )
                        else:
                            self.config.log.error("ProcessOutgoingData could not send via serial: Comand:{} speed:{} dir:{} distance:{} reply:{}".format(cmd, speed, direction, distance, reply_to_int) )
                
                elif cmd == Message.stop:
                    self.link.txBuff[0] = Message.stop
                    self.link.txBuff[1] = reply_to_int

                    if self.link.send(2):
                        self.config.log.info("ProcessOutgoingData sent via serial: Comand:{}".format(cmd) )
                    else:
                        self.config.log.error("ProcessOutgoingData could not send via serial: Comand:{}".format(cmd) )
                else:
                    self.config.log.error("ProcessOutgoingData : invalid comand:{}".format(cmd) )

            except Exception as e:
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                self.config.log.error("ProcessOutgoingData error " + str(e) + " line: " + str(exc_tb.tb_lineno))
                #print(exc_type, fname, exc_tb.tb_lineno)
        
        return



class ControllerInterface():
    def __init__(self, config):
        self.config = config
        self.name = ProcessNames.controller_interface
        
        # set on start()
        self.link = None
        self.message_bus = None

        return
    
    @staticmethod 
    def _worker(config):
        controller_interface = ControllerInterface(config)
        controller_interface.start()
        controller_interface.run()
        return

    @staticmethod
    def start_process(config):
        main_worker = mp.Process(target=ControllerInterface._worker, args=(config,))
        main_worker.start()
        return main_worker

    def start(self):
        try:
            self.config.log.info('The {} is starting. '.format(self.name))
            self.link = txfer.SerialTransfer(self.config["serial_port_a"], baud=self.config["serial_port_a_baud_rate"]) 
            self.link.open()
            self.config.log.info("serial port {} is open ".format(self.config["serial_port_a"]))
            self.message_bus = MessageBusManager.get_message_bus(self.config)
            self.config.log.info('The {} is ready. '.format(self.name))
        
        except ConnectionRefusedError as e:
            self.config.log.error("{} error. Unable to connect to message bus. Did you start bootstrap.py?".format(self.name))
            exit(-1)
        
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.config.log.error("{} error {} line: {}".format(self.name, str(e), str(exc_tb.tb_lineno)))
            exit(-1)
        return

    def run(self):

        self.config.log.info("Waiting for arduino to reboot" )
        time.sleep(3) # allow some time for the Arduino to completely reset

        # use to track to whom to send replys from the micro controller
        self.reply_to_table = ['dummy']  

        self.input_data_worker_thread = ProcessIncominData(self.config, self.link, self.message_bus, self.reply_to_table)
        self.input_data_worker_thread.start()
        self.send_items = queue.Queue()
        self.output_data_worker_thread = ProcessOutgoingData(self.config, self.link, self.send_items, self.reply_to_table)
        self.output_data_worker_thread.start()

        self.config.log.info('The {} is ready'.format(self.name))

        done = False
        while not done:
            try:
                msg = self.message_bus.receive(self.name)

                if (msg.cmd == Command.shutdown):
                    config.log.info("{} received shutdown command".format(self.name) )
                    done = True
                    break
                
                if (msg.cmd == Command.move):
                    self.send_items.put(msg)

                if (msg.cmd == Command.stop):
                    self.send_items.put(msg)


            except (KeyboardInterrupt, SystemExit):
                config.log.warning("{} received Ctrl+C ".format(self.name))
                break                

            except Exception as e: 
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                config.log.error("{} got error {} \n{} {}".format(self.name, str(e), exc_type, str(exc_tb.tb_lineno))) 
        
        self.config.log.info("shutting down {} process".format(self.name))
        self.link.close()
        #self.output_data_worker_thread.join()
        #self.input_data_worker_thread.join()
        self.config.log.info("{} terminated".format(self.name))
    
        return


if __name__ == '__main__':
    # debug only - this is started by bootstrap.py
    config = Config()
    controller_interface = ControllerInterface(config)
    controller_interface.start()
    controller_interface.run()    