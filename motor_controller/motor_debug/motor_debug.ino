#include <MotorH25.h>
#include "PulseCount.h"
#include "RobotAction.h"

MotorH25 motorL(23); //, 19, 18);
MotorH25 motorR(22); //, 20, 21);

PulseCount odometer;
RobotAction *robot_action;
RobotAction *do_nothing = new DoNothing();

void set_speed(float vl, float vr) {
    motorL.setPower(vl);
    motorR.setPower(vr);
}

void action_completed() {
  Serial3.println("Action Completed");
}
RobotAction *move_to = new MoveTo(set_speed, action_completed);

void stop()
{
  motorL.stop();
  motorR.stop();
}

void set_motor_power(float power, float direction)
{
  // -1 < power < 1  -1 < direction < 1
  // power = 0 -> stop   and direction = 0 -> straight

  float d = power * sin(0.5 * direction * 3.1415927);

  power = power > 0 ? power - abs(d) : power + abs(d);
  
  float powerr = power  - d/2.0;
  float powerl = power  + d/2.0;
  
  motorR.setPower(powerr);
  motorL.setPower(powerl);

 
}


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial3.begin(9600);
  Serial3.println("Motor Debug 2");

  motorL.initialize();
  motorR.initialize();
  odometer.begin();

  delay(10);
  motorL.setPower(0);
  motorR.setPower(0);
  delay(10);

  robot_action = do_nothing;
}




bool start_action = true;

void loop() {
    
  if (start_action)
  {
    robot_action = move_to;
    robot_action->begin();
    robot_action->set_parameters(200, 1.0, 0.8, FORWARD, FORWARD);
    start_action = false;
    odometer.reset();
  }
  
  robot_action->set_odometer(odometer.get_counts_left() , odometer.get_counts_right() );

  robot_action->loop();

  
  delay(20);  

}
