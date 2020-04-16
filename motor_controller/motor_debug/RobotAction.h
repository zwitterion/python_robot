/*
  RobotAction.h - Robot Operation Interface .
  Created by Jose Saura.
*/
#ifndef RobotAction_h
#define RobotAction_h

#define ACTION_NOT_STARTED 0
#define ACTION_RUNNING 1
#define ACTION_COMPLETED 2

#define FORWARD 1.0
#define REVERSE -1.0

// Base class
class RobotAction {
   public:
      // pure virtual function providing interface framework.
      virtual void begin() = 0;
      virtual void loop() = 0;

      void set_odometer(int left_count_, int right_count_) {
         left_count = left_count_;
         right_count = right_count_;
      }

      void set_parameters(int target_count_, float wheel_ratio_, float max_velocity_, float left_direction_, float right_direction_)
      {
         target_count = target_count_;
         wheel_ratio  = wheel_ratio_;
         max_velocity = max_velocity_;
         left_direction = left_direction_;
         right_direction = right_direction_;
      }
   
   protected:
      int left_count;
      int right_count;
      int target_count;
      int state;
      float wheel_ratio;
      float max_velocity;
      float left_direction=0;
      float right_direction=0;
      void (*motor_speed_cb) (float, float);
      void (*action_completed_cb)();
      int previous_error;

};
 
// All Operations
class DoNothing: public RobotAction {
   public:
      void begin() {

      }
      void loop() { 
         
      }
};

void report(int step_counter,float vl, float vr,int lcount, int rcount, float cutoff_start)
{
    Serial3.print(step_counter);
    Serial3.print("\t");
    Serial3.print(vl*100.0); 
    Serial3.print("\t");
    Serial3.print(vr*100.0); 
    Serial3.print("\t");
    Serial3.print(lcount);
    Serial3.print("\t");
    Serial3.print(rcount);
    Serial3.print("\t");
    Serial3.println(cutoff_start);
}

int sign(int x) {
   return x/abs(x); 
}


class MoveTo: public RobotAction {
   private:
      int step_counter;
      float kp = 0.05;
      float kd = 0.01;

   public:

      MoveTo()
      {

      }

      MoveTo(void (*motor_speed_cb_)(float, float), void (*action_completed_cb_)() ) 
      { 
         motor_speed_cb = motor_speed_cb_;
         action_completed_cb = action_completed_cb_;
      } 

      void begin() {

         state = ACTION_NOT_STARTED;
         left_count = 0;
         right_count = 0;
         step_counter = 0;
         previous_error = 1e4;
      }
      
      
      void loop() { 
         float v;
         float cutoff_start= -1;

         switch(state) {
            case ACTION_NOT_STARTED:
               Serial3.println("Starting Move Acion");
               state = ACTION_RUNNING;
               step_counter = 0;
               
               break;

            case ACTION_RUNNING:
               if ((left_count >= target_count || right_count > target_count))
               {
                     motor_speed_cb(0,0);
                     v=0;
                     state = ACTION_COMPLETED;
                     action_completed_cb();

               }
               else
               {
                  v = min(0.01 * step_counter, max_velocity);
                  
                  float odo_avg = (left_count + right_count) / 2.0;
                  cutoff_start = min(0.95 * target_count, max(target_count-20,0)); 
                  float lambda = 0.2;
                  if (odo_avg > cutoff_start && v > 0.4) {
                     // slow down...
                     v = max(v * pow(2, -lambda * (odo_avg - cutoff_start)), 0.4);
                  }

                  // correct left/right unbalance
                  int error = left_count * wheel_ratio  - right_count;
                  error = min(abs(error), 20) * sign(error);
                  if (previous_error >= 1e4) previous_error = error;
                  int error_deriv = error - previous_error;
                  float correction = error * kp + error_deriv * kd;

                  float vl = (v / (wheel_ratio+1.));
                  float vr = (v - vl);

                  vl = (1-correction) * vl;
                  vr = (1+correction) * vr;


                  motor_speed_cb(left_direction * vl, right_direction * vr);

                  report(step_counter,vl, vr, left_direction*left_count, right_direction*right_count, cutoff_start);

                  previous_error = error;
                  step_counter++;         
               }

            case ACTION_COMPLETED:
               break;

         }


      }
};


#endif