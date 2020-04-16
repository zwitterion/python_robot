/*
  MessageBus.h - Message Bus Manager.
  Created by Jose Saura.
*/
#ifndef MessageBus_h
#define MessageBus_h

#include "Arduino.h"
#include <cppQueue.h>
#include <SerialTransfer.h>
#include "Message.h"

#define MESSAGE_END    10 // line feed
#define MAX_MESSAGE_SIZE 100
#define NUM_CHANNELS 2

struct mbus_status_codes {
  int status;
  int state;
} ;


class MessageBus
{
  public:
    MessageBus();
    void send(unsigned char channel, Message* msg);
    bool receive(unsigned char channel, Message* msg);
    void begin(Stream &_port);
    void run();
    mbus_status_codes get_receive_status_code();

  private:
    unsigned char _next_queue;
    unsigned int message_index = 0;
    //Message msg;
    SerialTransfer serialTransfer;
        
    void transfer_in();
    void transfer_out();

    byte state = 0;                        // state machine current state
    char message[MAX_MESSAGE_SIZE+1];       // temporary buffer 

    Queue *_send_queues[NUM_CHANNELS];	
    Queue *_receive_queues[NUM_CHANNELS];
};

MessageBus::MessageBus()
{
  //_send_queues = new Queue*[num_channels];
  //_receive_queues = new Queue*[num_channels];
  message_index = 0;

  for (int i = 0; i < NUM_CHANNELS; ++i)
  {
    // assume in/out messgages are of the same size
    _send_queues[i] = new Queue(sizeof(Message),10,FIFO);
    _receive_queues[i] = new Queue(sizeof(Message),10,FIFO);
  }

}

void MessageBus::begin(Stream &_port)
{
    serialTransfer.begin(_port);
}

void MessageBus::send(unsigned char channel, Message* msg)
{
    _send_queues[channel]->push(msg);
}

bool MessageBus::receive(unsigned char channel, Message *msg)
{
    return _receive_queues[channel]->pop(msg);
}

void MessageBus::transfer_in()
{

  if(serialTransfer.available()) {
      Message msg;
      msg.parse_message(serialTransfer.rxBuff, &msg, serialTransfer.bytesRead);
      _receive_queues[0]->push(&msg);
  }

}

void MessageBus::transfer_out()
{
    // IF sizeof(Message) > txBuff =>  error
    // transfer a message from next queue
    //if (Serial.availableForWrite() >= 0) {             // if there uis room to send a message
      if (!_send_queues[_next_queue]->isEmpty()) {      // and if the queue has messages
        _send_queues[_next_queue]->pop(serialTransfer.txBuff);           // pop a message  
        
        serialTransfer.sendData(sizeof(Message));

      }
      _next_queue = (_next_queue+1)%NUM_CHANNELS;      // move to next queue
    //}


}

mbus_status_codes MessageBus::get_receive_status_code()
{
  //#return receive_error_code;
 
  mbus_status_codes result;
  result.status = serialTransfer.status;
  result.state = serialTransfer.state;
  
  return result;

}

void MessageBus::run()
{
    transfer_in();
    transfer_out();
}


#endif


