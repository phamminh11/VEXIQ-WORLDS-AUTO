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
motor LM = motor(PORT11, true);
motor RM = motor(PORT7, false);
motor IntakeMotorA = motor(PORT6, false);
motor IntakeMotorB = motor(PORT3, true);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);

/*vex-vision-config:end*/
motor ElevatorMotorA = motor(PORT1, true);
motor ElevatorMotorB = motor(PORT2, false);
motor_group Elevator = motor_group(ElevatorMotorA, ElevatorMotorB);
pneumatic Pneumatic4 = pneumatic(PORT4);
pneumatic Pneumatic5 = pneumatic(PORT5);

controller Controller = controller();



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "iq_cpp.h"

// Allows for easier use of the VEX Library
using namespace vex;
double PREV_TIME = 0, CURRENT_TIME = 0;
float MINSPEED = 0.0;

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
void RunIntake(float DELAY){
  int COUNT = 0;
  runIntake = true;
  wait(DELAY, seconds);
  while (runIntake){
    if(COUNT >= 50){
      Intake.spin(reverse);
      wait(0.5, seconds);
      Intake.spin(forward);
      COUNT = 0;
    }
    Intake.spin(forward);
    if(Intake.velocity(percent) == 0.0)
      COUNT+=1;
    else
      COUNT = 0;
    wait(5,msec);
  }
  Intake.stop();
}

void UpdateTimer(){
  while (true){
    CURRENT_TIME = Brain.Timer.value();
    wait(10, msec);
    PREV_TIME = CURRENT_TIME;
  }
}

class PID
{
public:
  float Kpr, Kdr, Kir;
  float Kpm, Kdm, Kim;
  void setupRotate(float p, float i, float d)
  {
    Kpr = p;
    Kir = i;
    Kdr = d;
  }
  void setupMove(float p, float i, float d)
  {
    Kpm = p;
    Kim = i;
    Kdm = d;
  }

  void turn(float heading)
  {
    float previousError = heading - BrainInertial.rotation(degrees);
    float integral = 0.0;
    int count = 0, maxCount = 10;
    RM.spin(forward);
    LM.spin(forward);
    while (count < maxCount)
    {
      float currentAngle = BrainInertial.rotation(degrees);
      float error = heading - currentAngle;
      integral += Kir * (error) * (CURRENT_TIME - PREV_TIME);
      float output = abs(Kpr * error + Kdr * (error - previousError) / (CURRENT_TIME - PREV_TIME) + integral) * 0.5;
      float leftVelocity, rightVelocity;
      if (currentAngle < heading)
        leftVelocity = output, rightVelocity = -output;
      else
        leftVelocity = -output, rightVelocity = output;
      if (rightVelocity < MINSPEED && rightVelocity > 0.0)
        rightVelocity = MINSPEED; // Make sure the robot is still moving very slowly
      if (leftVelocity < MINSPEED && leftVelocity > 0.0)
        leftVelocity = MINSPEED;
      if (rightVelocity > -MINSPEED && rightVelocity < 0.0)
        rightVelocity = -MINSPEED;
      if (leftVelocity > -MINSPEED && leftVelocity < 0.0)
        leftVelocity = -MINSPEED;
      LM.setVelocity(leftVelocity, percent);
      RM.setVelocity(rightVelocity, percent);
      if (abs(currentAngle - heading) <= 2.0)
        count += 1;
      else
        count = 0;
      wait(10, msec);
      previousError = error;
    }
    RM.stop();
    LM.stop();
  }

  void move(float distance, float accelDistance, float breakeDistance, float heading, float speed)
  {
    float previousError = heading - BrainInertial.rotation(degrees);
    float PREV_TICK_L, PREV_TICK_R;
    float originalDistance = distance;
    LM.setPosition(0, turns);
    RM.setPosition(0, turns);
    int count = 0, maxCount = 30;
    RM.spin(forward), LM.spin(forward);
    if (speed < 0)
      RM.spin(reverse), LM.spin(reverse);
    while (distance > 0 && count < maxCount)
    {
      PREV_TICK_L = LM.position(turns);
      PREV_TICK_R = RM.position(turns);
      float currentAngle = BrainInertial.rotation(degrees);
      float error = heading - currentAngle;
      float output = (Kpm * error + Kdm * (error - previousError) / (CURRENT_TIME - PREV_TIME)) * 0.5;
      float output_MUL = 1.0;
      if (originalDistance >= distance && distance >= (originalDistance - accelDistance) )
      {
        output_MUL = (originalDistance - distance) / accelDistance;
        if(output_MUL < 0.5) output_MUL = 0.5;
      }
      else if (breakeDistance > distance && distance > 0.0)
        output_MUL = distance / breakeDistance;
      float rightVelocity = speed * output_MUL - output, leftVelocity = speed * output_MUL + output;
      if (rightVelocity < 40.0 && rightVelocity >= 0.0)
        rightVelocity = 40.0; // Make sure the robot is still moving very slowly
      if (leftVelocity < 40.0 && leftVelocity >= 0.0)
        leftVelocity = 40.0;
      if (rightVelocity > -40.0 && rightVelocity < 0.0)
        rightVelocity = -40.0;
      if (leftVelocity > -40.0 && leftVelocity < 0.0)
        leftVelocity = -40.0;
      RM.setVelocity(rightVelocity, percent);
      LM.setVelocity(leftVelocity, percent);
      if ((abs(LM.velocity(percent)) == 0.0 && abs(RM.velocity(percent)) == 0.0) || distance <= 10.0)
        count += 0;
      else
        count = 0;
      wait(10, msec);
      previousError = error;
      float moveD = abs((RM.position(turns) - PREV_TICK_R + LM.position(turns) - PREV_TICK_L) * 199.49) * cos((currentAngle - heading) / 180.0 * 3.1415926);
      distance -= moveD;
    }
    RM.stop();
    LM.stop();
  }
};

void Graphing(){
  while (true){
    double t = Brain.Timer.value();
    printf("%.3f", t);
    printf("    ");
    printf("%.3f", BrainInertial.rotation(degrees) );
    printf("\n");
    wait(0.1, seconds);
  }
}

float Kpr = 1.3, Kdr = 0.02, Kir = 0.45;
float Kpm = 1.8, Kdm  = 0.15, Kim = 0.0;
float targetAngle = 0.0;

int main() {
  Pneumatic4.retract(cylinder1);
  Pneumatic5.retract(cylinder1);
  Pneumatic4.retract(cylinder2);
  Pneumatic5.extend(cylinder2);
  MINSPEED = 6.0;
  Intake.setVelocity(100, percent);
  Elevator.setStopping(hold);
  Elevator.setPosition(0, degrees);
  Elevator.spinToPosition(10, degrees);
  LM.setStopping(brake); RM.setStopping(brake);
  LM.setMaxTorque(100, percent); RM.setMaxTorque(100, percent);
  // Begin project code
  thread updatetime = thread(UpdateTimer);
  BrainInertial.calibrate();
  wait(2.5, seconds);
  BrainInertial.setRotation(0, degrees);
  Brain.Screen.print("Calibrated");
  //thread printrot = thread(Graphing);

  PID pid; 
  pid.setupRotate(Kpr, Kir, Kdr);
  pid.setupMove(Kpm, Kim, Kdm);
  //pid.move(600.0, 200.0, 200.0, 0.0, 60.0);
  LM.spin(forward); RM.spin(forward);
  while (1){
    Brain.Screen.print(printToBrain_numberFormat(2), static_cast<float>(MINSPEED));
    Brain.Screen.newLine();
    LM.setVelocity(MINSPEED, percent); RM.setVelocity(-MINSPEED, percent);
    // Brain.Screen.print(printToBrain_numberFormat(2), static_cast<float>(Kpr));
    // Brain.Screen.print("  ");
    // Brain.Screen.print(printToBrain_numberFormat(2), static_cast<float>(Kdr));
    // Brain.Screen.print("  ");
    // Brain.Screen.print(printToBrain_numberFormat(2), static_cast<float>(Kir));
    // Brain.Screen.newLine();
    if(Controller.AxisA.position() > 20.0){
      MINSPEED += 0.02;
    }
    if(Controller.AxisA.position() < -20.0){
      MINSPEED -= 0.02;
    }
    if(Controller.ButtonEUp.pressing()) //KP INCREASE
      Kpr += 0.01;
    if(Controller.ButtonEDown.pressing()) //KP DECREASE
      Kpr -= 0.01;
    if(Controller.ButtonFUp.pressing()) //KD INCREASE
      Kdr += 0.01;
    if(Controller.ButtonFDown.pressing()) //KD DECREASE
      Kdr -= 0.01;
    if(Controller.AxisD.position() > 20.0)
      Kir += 0.01;
    if(Controller.AxisD.position() < -20.0)
      Kir -= 0.01;
  
    if(Controller.ButtonRUp.pressing()){ //turn RIGHT 90 DEGREE
      targetAngle += 90.0;
      pid.setupRotate(Kpr, Kir, Kdr);
      Brain.Screen.print("test");
      pid.turn(targetAngle);
    }
    if(Controller.ButtonLUp.pressing()){ //turn LEFT 90 DEGREE
      targetAngle -= 90.0;
      pid.setupRotate(Kpr, Kir, Kdr);
      pid.turn(targetAngle);
    }
    if(Controller.ButtonRDown.pressing()){
      pid.setupMove(Kpr, Kir, Kdr);
      pid.move(900.0, 1.0, 400.0, 0.0, 80.0);
    }
    if(Controller.ButtonLDown.pressing()){
      pid.setupMove(Kpr, Kir, Kdr);
      pid.move(900.0, 1.0, 400.0, 0.0, -80.0);
    }
    wait(0.05, seconds);
  }
}