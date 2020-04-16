/*
  sbus.h

  Copyright (c) 2017, Jose Saura

*/

#pragma once

#include <Arduino.h>

// sbus processing state 
#define READING_FRAME         1
#define FRAME_NOT_STARTED     2
#define END_OF_FRAME_DETECTED 3

// sbus protocol codes
#define END_OF_FRAME_CODE 0
#define START_OF_FRAME_CODE 15

class SBUS {
  public:

    void begin(HardwareSerial &port);   // Stream &port
    void read();
    
    uint16_t getChannel(uint8_t channelIndex);
    uint16_t getStatus();

  private:
	byte buffer[25];
	byte buffer_index = 0;
	byte sbus_status = 100; // last byte in frame
	byte sbus_frame_state =0;
	HardwareSerial *port; // = &Serial1;
	unsigned int channels[18];

};

void SBUS::begin(HardwareSerial &port_)
{
    port = &port_;
    port->begin(100000, SERIAL_8E2);
    sbus_frame_state = FRAME_NOT_STARTED;
}


//  Example: 
//
//  SBUS sbus;
//  setup() {
//    sbus.begin(Serial1);
//  }
//
//  SerialEvent1()  { 
//      sbus.read(); 
//  }
//
//

// should be called from main code when data is available to read
void SBUS::read()
{
 int i=0;
  
  if (sbus_frame_state == FRAME_NOT_STARTED) {
    // read until we find a '0' that indicates end of a frame
    while (port->available() && sbus_frame_state != END_OF_FRAME_DETECTED) {
      // get the new byte:
      if (port->read() == END_OF_FRAME_CODE) {
        // we found the end of frame marker
        sbus_frame_state = END_OF_FRAME_DETECTED;
        //Serial.println("]");
      }
    }
  }
    
  if (sbus_frame_state == END_OF_FRAME_DETECTED) {
    // look for start frame marker
    if (port->available()) {
      // get the new byte:
      byte b = port->read();
      if (b == START_OF_FRAME_CODE) {
        // we found the start of frame marker
        sbus_frame_state = READING_FRAME;
        buffer_index = 0;
        for (i=0;i<22; i++)
          buffer[i] = 0;
        //Serial.print("[");
      }
      else {
        // if after an end of frame we did not find a start 
        // start all over
        sbus_frame_state = (b==END_OF_FRAME_CODE) ? END_OF_FRAME_DETECTED : FRAME_NOT_STARTED;
        //Serial.print("?");
        //Serial.println(b);
      }
    }
  }


  if (sbus_frame_state == READING_FRAME) {
     
     // read 23 bytes 
     while (port->available() && buffer_index < 24) { //q23
       buffer[buffer_index++] = port->read();
     }
          
     if (buffer_index == 24) 
     {
       if (buffer[23] == 0) {  // found end of frame
	       buffer_index = 0;

	       channels[0] = (buffer[1] << 8 | buffer[0])  & 0x07FF;                               //     8,  0
	       channels[1] = (buffer[2]<<5  | buffer[1]>>3) & 0x07FF;                              //     5,  3
	       channels[2] = (buffer[4]<<10 | buffer[3]<<2 | buffer[2]>>6 ) & 0x07FF;              // 10, 2,  6  
	       channels[3] = (buffer[5]<<7  | buffer[4]>>1 ) & 0x07FF;                             //     7,  1
	       channels[4] = (buffer[6]<<4  | buffer[5]>>4 ) & 0x07FF;                             //     4,  4
	       channels[5] = (buffer[8]<<9  | buffer[7]<<1 | buffer[6]>>7)  & 0x07FF;              // 9,  1,  7 
	       channels[6] = (buffer[9]<<6  | buffer[8]>>2 ) & 0x07FF;                             //     6,  2
	       channels[7] = (buffer[10] << 3 | buffer[9] >> 5)  & 0x07FF;                         //     3,  5
	       
	       channels[8] = (buffer[12] << 8 | buffer[11])  & 0x07FF;                               //     8,  0
	       channels[9] = (buffer[13]<<5  | buffer[12]>>3) & 0x07FF;                              //     5,  3
	       channels[10] = (buffer[15]<<10 | buffer[14]<<2 | buffer[13]>>6 ) & 0x07FF;              // 10, 2,  6  
	       channels[11] = (buffer[16]<<7  | buffer[15]>>1 ) & 0x07FF;                             //     7,  1
	       channels[12] = (buffer[17]<<4  | buffer[16]>>4 ) & 0x07FF;                             //     4,  4
	       channels[13] = (buffer[19]<<9  | buffer[18]<<1 | buffer[17]>>7)  & 0x07FF;              // 9,  1,  7 
	       channels[14] = (buffer[20]<<6  | buffer[19]>>2 ) & 0x07FF;                             //     6,  2
	       channels[15] = (buffer[21] << 3 | buffer[20] >> 5)  & 0x07FF;                         //     3,  5
      
               // last two digital channels encoded in last byte
               channels[16] = buffer[22] & 0x80; // test this
	       channels[17] = buffer[22] & 0x40; // test this

	       sbus_status = buffer[22] & 0x3F;   // i am seeing codes 4 (sporadically) and 12 (transmitter is turned off)

   	     sbus_frame_state = END_OF_FRAME_DETECTED;

       } 
       else {
	sbus_frame_state = FRAME_NOT_STARTED;
       }
 
/*
       for (i=0;i<3; i++)
       {
          Serial.print(buffer[i]); 
          Serial.print(" ");
          
       }

       Serial.print(" c ");
       Serial.print(channels[0]); 
       Serial.print(" ");
       Serial.print(channels[1]); 
       
       for (i=0;i<8; i++)
       {
//         Serial.print(channels[i]); 
  //       Serial.print(" ");
       }
       Serial.println("");
*/

     }
     
  }

}  

uint16_t SBUS::getChannel(uint8_t channelIndex)
{
   return channels[channelIndex-1];
}


uint16_t SBUS::getStatus()
{
   return sbus_status;	
}


