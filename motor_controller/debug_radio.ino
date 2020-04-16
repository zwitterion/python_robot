#include "SBUS/sbus.h"

SBUS sbus;

uint16_t get_speed() {
  uint16_t v = sbus.getChannel(3);
  return v;
}

uint16_t get_direction() {
  uint16_t v = sbus.getChannel(2);
  return v;
}

int16_t get_reset() {
  int16_t v = sbus.getChannel(7);
  if (v>1500) v=1; else if (v>300) v=0; else v = -1;
  return v;
}

void serialEvent2() {
  sbus.read();
}


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  mbus.begin(Serial);

  // initialize radio (Futaba) line
  Serial2.begin(100000, SERIAL_8E2);
  sbus.begin(Serial2);

  
}



// the loop routine runs over and over again forever:
void loop() {
  
  Serial.print(get_speed());
  Serial.print(get_direction());

  delay(1);  
    
  
}
