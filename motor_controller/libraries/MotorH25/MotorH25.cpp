#include "MotorH25.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"



MotorH25::MotorH25(uint8_t pinPowerController_)
{
	pinPowerController = pinPowerController_;
}

void MotorH25::initialize()
{
	pinMode(pinPowerController, OUTPUT);
	digitalWrite(pinPowerController, LOW);
	stop();
}

// sets current power by sending a pulse to the controller
int MotorH25::setPower(float power_) {
	/*if (power_ == power)
	{
		// nothing to do;
		return 0;
	}*/

	if (power_ <-1.0 || power_ > 1.0)
	{
		power = 0;
	}
	
	power = power_;
	sendPulse(power);
	return 0;
}

// returns last power value sent via setPower()
float MotorH25::getPower()
{
	return power;
}
void MotorH25::stop()
{
	sendPulse(0);
	digitalWrite(pinPowerController, LOW);
	power = 0;
}

void MotorH25::sendPulse(float p)
{
	// Pulse Input: 1.0ms Full Reverse, 1.5ms Neutral (off), 2.0ms Full Forward
	// https://www.parallax.com/product/29144
	//  -1 < p < 1  =>  1000 2000
	// p=0 -> stop
	noInterrupts();
	digitalWrite(pinPowerController, HIGH);
	int pulse_length = POWER_CONVERT_B + (p)* POWER_CONVERT_A;
	delayMicroseconds(pulse_length);
	digitalWrite(pinPowerController, LOW);
	interrupts();
}



//void MotorH25::loop(float targetSpeed) {

	//if (millis() - lastPulse > 20 || v == 0) // only send one pulse every 20 ms
	//{
	//	sendPulse(speed);
		//Serial.println(speed);
	//}


	//long milisec = millis();
	

//}
