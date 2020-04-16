#ifndef PulseCount_h
#define PulseCount_h

#include <Arduino.h>
#include <inttypes.h>
#include <stdio.h>

class PulseCount {
public:
  PulseCount();
  void begin();
  unsigned long get_counts_left();
  unsigned long get_counts_right();
  void reset();

  char get_direction_left();
  char get_direction_right();
  

private:
  uint8_t x;
  

};



volatile unsigned long pulse_counter_18 = 0;
volatile unsigned long pulse_counter_19 = 0;
volatile unsigned long pulse_counter_20 = 0;
volatile unsigned long pulse_counter_21 = 0;
  
volatile unsigned long time_18_19 = 0;
volatile unsigned long time_20_21 = 0;

// -1 backwards +1 forwards 0 unknown
volatile char direction_18_19 = 0;
volatile char direction_20_21 = 0;


  
void handleInterrupt18() {

  if (pulse_counter_18 > pulse_counter_19)
	direction_18_19=-1;
  else
	direction_18_19=+1;

  pulse_counter_18++;

//  time_18_19 = millis();
}
void handleInterrupt19() {
  pulse_counter_19++;
}

void handleInterrupt20() {
  if (pulse_counter_20 > pulse_counter_21)
	direction_20_21=-1;
  else
	direction_20_21=+1;

  pulse_counter_20++;
}

void handleInterrupt21() {
  pulse_counter_21++;
}


PulseCount::PulseCount()
{
}

void PulseCount::begin()
{
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), handleInterrupt18, RISING);
  attachInterrupt(digitalPinToInterrupt(19), handleInterrupt19, RISING);
  attachInterrupt(digitalPinToInterrupt(20), handleInterrupt20, RISING);
  attachInterrupt(digitalPinToInterrupt(21), handleInterrupt21, RISING);
}

void PulseCount::reset()
{
  pulse_counter_18 = 0;
  pulse_counter_19 = 0;
  pulse_counter_20 = 0;
  pulse_counter_21 = 0;
}

unsigned long PulseCount::get_counts_left()
{
  return pulse_counter_18;
}
unsigned long PulseCount::get_counts_right()
{
  return pulse_counter_20;
}

char PulseCount::get_direction_left()
{
  return direction_18_19;
}

char PulseCount::get_direction_right()
{
  return direction_20_21;
}


#endif
