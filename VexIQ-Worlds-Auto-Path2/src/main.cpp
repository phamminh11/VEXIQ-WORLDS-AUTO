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
#define waitUntil(condition) \
  do                         \
  {                          \
    wait(5, msec);           \
  } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// Robot configuration code.
inertial BrainInertial = inertial();
motor LM = motor(PORT4, true);
motor RM = motor(PORT11, false);
motor IntakeMotorA = motor(PORT1, false);
motor IntakeMotorB = motor(PORT8, true);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);

/*vex-vision-config:end*/
motor ElevatorMotorA = motor(PORT9, true);
motor ElevatorMotorB = motor(PORT3, false);
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
double previousTime = 0, currentTime = 0;
float minRotSpeed = 0.0, minMoveSpeed = 0.0;


bool runIntake = false;
void RunIntake()
{
  if(!runIntake){
    runIntake = true;
    Intake.spin(forward);
    Elevator.spin(forward);
  }
  else{
    runIntake = false;
    Intake.stop();
    Elevator.stop();
  }
}

void UpdateTimer()
{
  while (true)
  {
    currentTime = Brain.Timer.value();
    wait(10, msec);
    previousTime = currentTime;
  }
}

bool releasePurple = false;
void PurpleStorage(){
  if(!releasePurple){
    releasePurple = true;
    Pneumatic4.extend(cylinder2);
    Pneumatic5.retract(cylinder2);
  }
  else{
    releasePurple = false;
    Pneumatic4.retract(cylinder2);
    Pneumatic5.extend(cylinder2);
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
    int count = 0, maxCount = 5;
    RM.spin(forward);
    LM.spin(forward);
    while (count < maxCount)
    {
      float currentAngle = BrainInertial.rotation(degrees);
      float error = heading - currentAngle;
      integral += Kir * (error) * (currentTime - previousTime);
      float output = abs(Kpr * error + Kdr * (error - previousError) / (currentTime - previousTime) + integral) * 0.5;
      float leftVelocity, rightVelocity;
      if (currentAngle < heading)
        leftVelocity = output, rightVelocity = -output;
      else
        leftVelocity = -output, rightVelocity = output;
      if (rightVelocity < minRotSpeed && rightVelocity > 0.0)
        rightVelocity = minRotSpeed; // Make sure the robot is still moving very slowly
      if (leftVelocity < minRotSpeed && leftVelocity > 0.0)
        leftVelocity = minRotSpeed;
      if (rightVelocity > -minRotSpeed && rightVelocity < 0.0)
        rightVelocity = -minRotSpeed;
      if (leftVelocity > -minRotSpeed && leftVelocity < 0.0)
        leftVelocity = -minRotSpeed;
      LM.setVelocity(leftVelocity, percent);
      RM.setVelocity(rightVelocity, percent);
      if (abs(currentAngle - heading) <= 2.5)
        count += 1;
      else
        count = 0;
      wait(10, msec);
      previousError = error;
    }
    RM.setVelocity(0, percent);
    LM.setVelocity(0, percent);
  }

  void move(float distance, float accelDistance, float breakeDistance, float heading, float speed)
  {
    float previousError = heading - BrainInertial.rotation(degrees);
    float PREV_TICK_L, PREV_TICK_R;
    float originalDistance = distance;
    LM.setPosition(0, degrees);
    RM.setPosition(0, degrees);
    int count = 0, maxCount = 5;
    RM.spin(forward), LM.spin(forward);
    if (speed < 0)
      RM.spin(reverse), LM.spin(reverse);
    while (distance > 0 && count < maxCount)
    {
      PREV_TICK_L = LM.position(turns);
      PREV_TICK_R = RM.position(turns);
      float currentAngle = BrainInertial.rotation(degrees);
      float error = heading - currentAngle;
      float output = (Kpm * error + Kdm * (error - previousError) / (currentTime - previousTime)) * 0.5;
      float output_MUL = 1.0;
      if (originalDistance >= distance && distance >= (originalDistance - accelDistance))
      {
        output_MUL = (originalDistance - distance) / accelDistance;
        if (output_MUL < 0.6)
          output_MUL = 0.6;
      }
      else if (breakeDistance >= distance && distance >= 0.0)
        output_MUL = distance / breakeDistance;
      float rightVelocity = speed * output_MUL - output, leftVelocity = speed * output_MUL + output;
      if (rightVelocity < minMoveSpeed && rightVelocity >= 0.0)
        rightVelocity = minMoveSpeed; // Make sure the robot is still moving very slowly
      if (leftVelocity < minMoveSpeed && leftVelocity >= 0.0)
        leftVelocity = minMoveSpeed;
      if (rightVelocity > -minMoveSpeed && rightVelocity < 0.0)
        rightVelocity = -minMoveSpeed;
      if (leftVelocity > -minMoveSpeed && leftVelocity < 0.0)
        leftVelocity = -minMoveSpeed;
      RM.setVelocity(rightVelocity, percent);
      LM.setVelocity(leftVelocity, percent);
      if (( abs(LM.velocity(percent)) == 0.0 && abs(RM.velocity(percent)) == 0.0 && distance < (originalDistance-50.0) ) || distance <= 20.0)
        count += 1;
      else
        count = 0;
      wait(10, msec);
      previousError = error;
      float moveD = abs((RM.position(turns) - PREV_TICK_R + LM.position(turns) - PREV_TICK_L) * 199.49) * cos((currentAngle - heading) / 180.0 * 3.1415926);
      distance -= moveD;
    }
    RM.setVelocity(0, percent);
    LM.setVelocity(0, percent);
  }
};

float Kpr = 1.05, Kdr = 0.03, Kir = 0.0; //Default PID rotation value
float Kpm = 20.0, Kdm = 0.0, Kim = 0.0; //Defailt PID movement value
float defaultAccelDistance = 500.0;
float defaultBrakeDistance = 200.0;
PID pid;

void clearSupplyZone()
{
  pid.setupMove(5.0, Kim, Kdm);
  pid.move(400.0, -1.0, 200.0, 90.0, 40.0);
  wait(0.5, seconds);
  pid.move(200.0, -1.0, 200.0, 90.0, 40.0);
  wait(0.5, seconds);
  pid.move(150.0, -1.0, 50.0, 90.0, -80.0);
}

void setup()
{
  minRotSpeed = 9.3;
  minMoveSpeed = 50.0;
  Pneumatic4.retract(cylinder1);
  Pneumatic5.retract(cylinder1);
  Pneumatic4.retract(cylinder2);
  Pneumatic5.extend(cylinder2);
  Intake.setVelocity(100, percent);
  Elevator.setStopping(hold);
  Elevator.setPosition(0, degrees);
  LM.setStopping(brake);
  RM.setStopping(brake);
  LM.setMaxTorque(100, percent);
  RM.setMaxTorque(100, percent);
  Intake.setMaxTorque(100, percent);
  Elevator.setMaxTorque(100, percent);
  thread updatetime = thread(UpdateTimer);
  BrainInertial.calibrate();
  wait(2.5, seconds);
  BrainInertial.setRotation(0, degrees);
  Brain.Screen.print("Calibrated");
  pid.setupRotate(Kpr, Kir, Kdr);
  pid.setupMove(Kpm, Kim, Kdm);
}

void time(){
  while (true){
    Brain.Screen.print("%.3f", Brain.Timer.value());
    wait(10, msec);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
  }
}

int main()
{
  setup();
  //Autonomous path
  thread time_ = thread(time);
  RunIntake();
  pid.move(1250.0, -10.0, 200.0, 0.0, 100.0);
  pid.turn(90.0);
  //Supply zone phase
  clearSupplyZone();
  //Goal 1
  // pid.turn(135.0);
  // pid.move(500.0, 10.0, -10.0, 135.0, 100.0);
  // pid.setupMove(2.8, Kim, Kdm);
  // pid.move(400.0, -1.0, -200.0, 210.0, 100.0);
  // pid.setupMove(2.6, Kim, Kdm);
  // pid.move(300.0, -1.0, 200.0, 290.0, 100.0);
  // pid.setupMove(1.4, Kim, Kdm);
  // pid.move(1000.0, -1.0, 200.0, 280.0, -100.0);

  // //Goal 2:
  // pid.setupMove(3.0, Kim, Kdm);
  // pid.move(1603.317, -1.0, -300.0, 300.0, 100.0);
  // pid.setupMove(1.2, Kim, Kdm);
  // pid.move(600.0, -1.0, 200.0, 370.0, 100.0);
  // pid.move(3000.0, -1.0, 400.0, 10.0, -80.0);

  // //Goal 3:
  // pid.setupMove(3.0, Kim, Kdm);
  // pid.move(1000.0, -1.0, -200.0, 30.0, 100.0);
  // pid.setupMove(1.8, Kim, Kdm);
  // pid.move(300.0, -1.0, -100.0, 70.0, 100.0);
  // pid.move(1200.0, -1.0, 100.0, 90.0, 100.0);
}