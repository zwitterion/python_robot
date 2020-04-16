#ifndef H25_h
#define H25_h

#include <Arduino.h>
#include <inttypes.h>

// commands
#define POWER_CONVERT_A -500.0
#define POWER_CONVERT_B 1480.0

class MotorH25 {
public:
  MotorH25(uint8_t pinPowerController_);
  int setPower(float power_);
  float getPower();
  void stop();
  void initialize();
private:
  uint8_t pinPowerController;
  float power = -100; // set to invalid value to force setPower to run even if passed 0 the first time

  void sendPulse(float p);

};

#endif
