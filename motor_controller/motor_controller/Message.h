/*
  Message.h - Standard Message .
  Created by Jose Saura.
*/
#ifndef Message_h
#define Message_h

#define COMMAND_STATUS 1
#define COMMAND_FUTABA 2
#define COMMAND_ODOMETER 3
#define COMMAND_STOP 5
#define COMMAND_MOVE 100      // obsolete use COMMAND_MOVE_TO
#define COMMAND_MOVE_TO 101

#define COMMAND_COMPLETED 200

class Message
{
  public:
    void parse_message(char *buffer, Message *msg, int length) {
        length = min(length, sizeof(Message));
        memcpy(msg, buffer, length);
    }

  public:
    unsigned char command;
    union {
        struct {
            int speed;
            int direction;
            unsigned int distance;
            unsigned char reply_to;
        } move;
         struct {
            unsigned char reply_to;
        } stop;
        struct {
            int status;
            int channel[16];
        } futaba;
        struct {
            unsigned int left;
            unsigned int right;
        } odometer;
        struct {
            char text[20];
        } status;
        struct {
          unsigned char reply_to;
          unsigned char status;
          unsigned int left;
          unsigned int right;
        } command_completed;
    } params;
};





#endif