#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
motor LM = motor(PORT1, true);
motor RM = motor(PORT7, false);
motor IntakeMotorA = motor(PORT2, false);
motor IntakeMotorB = motor(PORT8, false);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "iq_cpp.h"

// Allows for easier use of the VEX Library
using namespace vex;
double prevTime = 0, curTime = 0;

const char* printToBrain_numberFormat(int precision) {
  // look at the current precision setting to find the format string
  switch(precision){
    case 0:  return "%.0f"; // 0 decimal places (1)
    case 1:  return "%.1f"; // 1 decimal place  (0.1)
    case 2:  return "%.2f"; // 2 decimal places (0.01)
    case 3:  return "%.3f"; // 3 decimal places (0.001)
    default: return "%f"; // use the print system default for everthing else
  }
}

bool runIntake = false;
void RunIntake(){
  int dem = 0;
  while (runIntake){
    if(dem == 30){
      Intake.spin(reverse);
      wait(0.4, seconds);
      Intake.spin(forward);
    }
    Intake.spin(forward);
    if(Intake.velocity(percent) == 0.0)
      dem+=1;
    else
      dem = 0;
    wait(5,msec);
  }
  Intake.stop();
}
void StopIntake(){
  runIntake = false;
}

void UpdateTimer(){
  while (true){
    curTime = Brain.Timer.value();
    wait(5, msec);
    prevTime = curTime;

  }
}

void GyroTurn(float angle, float Kp, float Kd){
  float prevError = angle-BrainInertial.rotation(degrees);-1.0;
  int dem = 0, maxWaitTime = 20;
  LM.setPosition(0, degrees);
  RM.spin(forward); LM.spin(forward);
  while ( dem < maxWaitTime ){
    float cur_angle = BrainInertial.rotation(degrees);
    float error = angle-cur_angle;
    float output = abs(Kp * error + Kd*(error-prevError)/ (curTime-prevTime));
    Brain.Screen.print(printToBrain_numberFormat(2), static_cast<float>(LM.position(degrees)) );
    Brain.Screen.newLine();
    output *= 0.5;
    if(cur_angle < angle){
      RM.setVelocity(-output, percent);
      LM.setVelocity(output, percent);
    }
    else{
      RM.setVelocity(output, percent);
      LM.setVelocity(-output, percent);
    }
    if( abs(cur_angle - angle) <= 1.0)
      dem+=1;
    else
      dem=0;
    wait(1,msec);
    prevError = error;
  }
  RM.stop(); LM.stop();
}

void GyroMove(float dist, float brake_dist, float angle, float speed, float Kp, float Kd){
  float prevError = angle-BrainInertial.rotation(degrees);-1.0;
  float prevTickL,prevTickR;
  LM.setPosition(0, degrees);
  RM.setPosition(0, degrees);
  int dem = 0, maxWaitTime = 100;
  RM.spin(forward), LM.spin(forward);
  if(speed < 0)
    RM.spin(reverse), LM.spin(reverse);
  while (dist > 0 && dem < maxWaitTime){
    prevTickL = LM.position(turns); 
    prevTickR = RM.position(turns);
    float curGoc = BrainInertial.rotation(degrees);;
    float error = angle - curGoc;
    float output = Kp * error + Kd*(error-prevError)/ (curTime-prevTime);
    float output_speed_ratio = dist/brake_dist;
    if(output_speed_ratio > 1.0) output_speed_ratio = 1.0;
    RM.setVelocity(speed*output_speed_ratio-output, percent);
    LM.setVelocity(speed*output_speed_ratio+output, percent);
    if(abs(LM.velocity(percent)) < 2.0 && abs(RM.velocity(percent)) < 2.0)
      dem+=1;
    else
      dem = 0;
    wait(0.01,seconds);
    prevError = error;
    float moved = abs( (RM.position(turns) - prevTickR + LM.position(turns) - prevTickL) * 199.49 ) * cos((curGoc-angle)/180.0 * 3.1415926 );
    dist -= moved;
  }
  RM.stop(); LM.stop();
}

int main() {
  // Begin project code
  LM.setStopping(brake);
  RM.setStopping(brake);
  thread updatetime = thread(UpdateTimer);
  BrainInertial.calibrate();
  wait(3, seconds);
  Brain.Screen.print("Calibrated");
  GyroMove(900.0, 50.0, 0.0, 90.0, 1.5, 0.5);
  GyroTurn(90.0, 1.1, 0.2);
  GyroMove(1200.0, 50.0, 90.0, 90.0, 1.5, 0.1);
}