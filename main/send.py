import os
import multiprocessing as mp
from multiprocessing.managers import BaseManager
import logging

from config import Config

class MessageBusManager(BaseManager):
    pass

def worker(config):
    mp.current_process().authkey = config['authkey']
    mp.log_to_stderr(logging.DEBUG)
    config.log.info('The executor is starting. ' + str(os.getpid()))

    MessageBusManager.register('mbus') # MessageBus, MessageBusProxy)
    message_bus_manager = MessageBusManager(address=('127.0.0.1', 50000), authkey=config['authkey']) # address=('foo.bar.org', 50000), authkey=b'abracadabra'
    message_bus_manager.connect()
   
    message_bus = message_bus_manager.mbus(None)
    
    message_bus.send("executor", "debug")
    message_bus.send("director", "shutdown")

    return


if __name__ == '__main__':
    config = Config()
    executor_worker = mp.Process(target=worker, args=(config,))
    executor_worker.start()
    executor_worker.join()
    print("end")