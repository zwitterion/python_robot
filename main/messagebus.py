
import multiprocessing as mp
from multiprocessing.managers import BaseManager, BaseProxy, MakeProxyType, SyncManager
import signal 

from multiprocessing import Queue

from config import Config
from message import Message

class MessageBus():
    
    queues = {} # {"executor": mp.Queue(), "main":mp.Queue() }

    def __init__(self, config, *args, **kwargs):
        #super().__init__(*args, **kwargs)
        if (config is None): return
        
        # set static variables
        MessageBus.max_queue_size = config["message_bus_max_queue_size"]
        MessageBus.config = config
        MessageBus.config.log.info('The message bus has been initialized.')
        MessageBus.subscribers = {}

    def create_topic(self, topic):
        """ initialize a topic/DL
        """
        if topic not in MessageBus.subscribers:
            # need lock here
            MessageBus.subscribers[topic] = []

        MessageBus.config.log.info('Created topic {}'.format(topic))


    def subscribe(self, subscriber, topic):
        """ subscribe <subscriber> to a topic 
        """
        if topic not in MessageBus.subscribers:
            # need lock here
            MessageBus.config.log.info('Creating topic {}'.format(topic))
            MessageBus.subscribers[topic] = []

        topic_subscribers = MessageBus.subscribers[topic]

        if subscriber not in topic_subscribers:
            topic_subscribers.append(subscriber)

        MessageBus.config.log.info('{} subscribed to {}'.format(subscriber, topic))

    def send(self, address, msg):

        if address in MessageBus.subscribers:
            # to_address is a topic
            receipients = MessageBus.subscribers[address]
            MessageBus.config.log.info('Message published to topic: {}  # receipients: {}'.format(address, len(receipients)))
            print(receipients)
        else:
            receipients = [address]

        for to_address in receipients:
            try:
                if to_address not in MessageBus.queues:
                    # potential race condition - need to add lock/etc
                    # better to preconfigure all queues above than using this...
                    MessageBus.queues[to_address] = mp.Queue()
                    MessageBus.config.log.info('Message queued created for ' + to_address)


                q = MessageBus.queues[to_address]
                while q.qsize() > MessageBus.max_queue_size:
                    MessageBus.config.log.info('Queue Size exceeded {} for queue {} - removing old items'.format(MessageBus.max_queue_size, to_address))
                    q.get()

                q.put(msg.to_json())
                MessageBus.config.log.info('Message has been queued for: {} qsize: {}'.format(to_address,q.qsize()))
            except Exception as e:
                MessageBus.config.log.error('Message bus: send error:' + str(e))
        return
    
    def receive(self, from_address, latest_message=False):
        if from_address not in MessageBus.queues:
            # potential race condition - need to add lock/etc
            # better to preconfigure all queues above than using this...
            MessageBus.queues[from_address] = mp.Queue()
            MessageBus.config.log.info('(r) Message queued created for ' + from_address)

        q = MessageBus.queues[from_address]
        msg = None
        try:
            if latest_message:
                # if consumer only wants latest message, discard all but one
                while q.qsize() > 1:
                    q.get()

            msg = q.get()
            MessageBus.config.log.info('Message popped. ' + from_address)
            msg = Message.from_json(msg)
            MessageBus.config.log.info('Message parsed ' + str(msg.cmd))
            #print("****QUEUEs LENGTH:" + str(len(MessageBus.queues)))
            #for k in MessageBus.queues.keys():
            #    print(k)

            return msg

        except (KeyboardInterrupt, SystemExit):
            print("Abort!-1")
            raise

        #message_handler(msg)
        return None

    def clear(self, address):
        if address in MessageBus.queues:
            q = MessageBus.queues[address]
            i=0
            while q.qsize() > 0:
                q.get()
                i+=1
            MessageBus.config.log.info('{} pending messages cleared from queue {}'.format(i, address))

# required for proper handling of Ctrl+C
class KeyboardInterruptError(Exception): pass

class MessageBusProxy(BaseProxy):
    _exposed_ = ('send', 'receive', "subscribe", "create_topic", "clear")

    def send(self, *args):
        return self._callmethod('send', args)

    def receive(self, *args):
        def signal_handle(_signal, frame):
            raise KeyboardInterruptError()

        # required for proper handling of Ctrl+C
        signal.signal(signal.SIGINT, signal_handle)
        try: 
            r = self._callmethod('receive', args)
            return r

        except KeyboardInterruptError:
            raise KeyboardInterrupt

    def create_topic(self, *args):
        return self._callmethod('create_topic', args)

    def subscribe(self, *args):
        return self._callmethod('subscribe', args)
        
    def clear(self, *args):
        return self._callmethod('clear', args)


if __name__=='__main__':

    print("main")