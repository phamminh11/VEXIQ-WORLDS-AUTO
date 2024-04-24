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

motor ElevatorMotorA = motor(PORT9, true);
motor ElevatorMotorB = motor(PORT3, false);
motor_group Elevator = motor_group(ElevatorMotorA, ElevatorMotorB);

pneumatic Pneumatic4 = pneumatic(PORT2);
pneumatic Pneumatic5 = pneumatic(PORT7);

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
  if (!runIntake)
  {
    runIntake = true;
    Intake.spin(forward);
    Elevator.spin(forward);
  }
  else
  {
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
void PurpleStorage()
{
  if (!releasePurple)
  {
    releasePurple = true;
    Pneumatic4.retract(cylinder2);
    Pneumatic5.extend(cylinder2);
  }
  else
  {
    releasePurple = false;
    Pneumatic4.extend(cylinder2);
    Pneumatic5.retract(cylinder2);
  }
}

bool two_to_one = false;
void SwitchGear()
{
  if (!two_to_one)
  {
    two_to_one = true;
    Pneumatic4.retract(cylinder1);
    Pneumatic5.retract(cylinder1);
  }
  else
  {
    two_to_one = false;
    Pneumatic4.extend(cylinder1);
    Pneumatic5.extend(cylinder1);
  }
}

void GreenStorage(int level)
{
  int _turns;
  switch (level)
  {
  case 1:
    _turns = 1;
    break;
  case 2:
    _turns = 5;
    break;
  case 3:
    _turns = 20;
    break;
  default:
    _turns = 20;
    break;
  }
  RunIntake();
  Elevator.setMaxTorque(100, percent);
  Elevator.spin(reverse);
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

  void turn(float heading, float cnt = 9999.0)
  {
    float previousError = heading - BrainInertial.rotation(degrees);
    float integral = 0.0;
    int count = 0, maxCount = 5;
    float count2 = 0.0;
    RM.spin(forward);
    LM.spin(forward);
    while (count < maxCount && count2 <= cnt)
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
      if (abs(currentAngle - heading) <= 5)
        count += 1;
      else
        count = 0;
      count2 += 0.01;
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
      if ((abs(LM.velocity(percent)) == 0.0 && abs(RM.velocity(percent)) == 0.0 && distance < (originalDistance - 50.0)) || distance <= 20.0)
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

float Kpr = 1.05, Kdr = 0.03, Kir = 0.0; // Default PID rotation value
float Kpm = 20.0, Kdm = 0.0, Kim = 0.0;  // Defailt PID movement value
float defaultAccelDistance = 500.0;
float defaultBrakeDistance = 200.0;
PID pid;

void clearSupplyZone()
{
  pid.setupMove(5.0, Kim, Kdm);
  pid.move(2000.0, -1.0, 200.0, 100.0, 100.0);
  LM.spin(forward);
  RM.spin(forward);
  LM.setVelocity(-100, percent);
  RM.setVelocity(100, percent);
  wait(0.5, seconds);
  LM.setVelocity(100, percent);
  RM.setVelocity(-100, percent);
  wait(0.5, seconds);
  LM.setVelocity(-100, percent);
  RM.setVelocity(100, percent);
  wait(0.5, seconds);
  LM.setVelocity(100, percent);
  RM.setVelocity(-100, percent);
  wait(0.5, seconds);
  LM.stop();
  RM.stop();

  pid.move(100.0, -1.0, 00.0, 90.0, -100.0);
  pid.move(2000.0, -1.0, 200.0, 90.0, 100.0);
  LM.spin(forward);
  RM.spin(forward);
  LM.setVelocity(-100, percent);
  RM.setVelocity(100, percent);
  wait(0.5, seconds);
  LM.setVelocity(100, percent);
  RM.setVelocity(-100, percent);
  wait(0.5, seconds);
  LM.setVelocity(-100, percent);
  RM.setVelocity(100, percent);
  wait(1.0, seconds);
  LM.stop();
  RM.stop();

  pid.move(100.0, -1.0, 0.0, 90.0, -100.0);
  pid.move(2000.0, -1.0, 200.0, 90.0, 100.0);
  LM.spin(forward);
  RM.spin(forward);
  LM.setVelocity(-100, percent);
  RM.setVelocity(100, percent);
  wait(0.5, seconds);
  LM.stop();
  RM.stop();
  pid.move(200.0, -1.0, 200.0, 90.0, -100.0);

  minRotSpeed = 60.0;
  pid.turn(125.0, 1.5);

  // pid.move(2000.0, -1.0, 200.0, 90.0, 100.0);
  //  LM.spin(forward); RM.spin(forward);
  //  LM.setVelocity(-100, percent); RM.setVelocity(100, percent);
  //  wait(0.5, seconds);
  //  LM.setVelocity(100, percent); RM.setVelocity(-100, percent);
  //  wait(0.5, seconds);
  //  LM.setVelocity(-100, percent); RM.setVelocity(100, percent);
  //  wait(0.5, seconds);
  //  LM.setVelocity(100, percent); RM.setVelocity(-100, percent);
  //  wait(0.5, seconds);
  //  LM.stop(); RM.stop();

  // pid.move(100.0, -1.0, 0.0, 90.0, -100.0);
  pid.move(2000.0, -1.0, 200.0, 90.0, 100.0);
  pid.move(500.0, -1.0, 200.0, 90.0, -100.0);
}

void setup()
{
  minRotSpeed = 10.0;
  minMoveSpeed = 50.0;
  Pneumatic4.extend(cylinder1);
  Pneumatic5.extend(cylinder1);
  Pneumatic4.extend(cylinder2);
  Pneumatic5.retract(cylinder2);
  Intake.setVelocity(100, percent);
  Elevator.setVelocity(100, percent);
  Elevator.setStopping(hold);
  Elevator.setPosition(0, degrees);
  LM.setStopping(brake);
  RM.setStopping(brake);
  LM.setMaxTorque(100, percent);
  RM.setMaxTorque(100, percent);
  Intake.setMaxTorque(100, percent);
  Elevator.setMaxTorque(100, percent);
  thread updatetime = thread(UpdateTimer);
  BrainInertial.setRotation(0, degrees);
  Brain.Screen.print("Calibrated");
  pid.setupRotate(Kpr, Kir, Kdr);
  pid.setupMove(Kpm, Kim, Kdm);
}

void time()
{
  while (true)
  {
    Brain.Screen.print("%.3f", Brain.Timer.value());
    wait(10, msec);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
  }
}

int main()
{
  setup();
  // Autonomous path
  thread time_ = thread(time);
  RunIntake();
  pid.move(1100.0, -10.0, -10.0, 3.0, 100.0);
  // pid.move(200.0, -10.0, 10.0, 0.0, -100.0);
  pid.turn(90.0);
  pid.move(130.0, -10.0, 50.0, 90.0, -100.0); // May be optimized
  // Supply zone phase
  clearSupplyZone(); // Good enough but luck based and may need optimization
  // Goal 1
  minRotSpeed = 60.0;
  pid.turn(-25.0);
  pid.setupMove(2.0, Kim, Kdm);
  wait(0.3, seconds);
  pid.move(650.0, -1.0, 200.0, -70.0, 60.0);
  wait(0.5, seconds);
  pid.move(150.0, -1.0, 200.0, -80.0, 60.0);
  wait(0.5, seconds);
  pid.move(1000.0, -1.0, 200.0, -80.0, 60.0);
  wait(0.5, seconds);
  pid.setupMove(5.0, Kim, Kdm);
  pid.move(250.0, -1.0, 10.0, -75.0, -100.0);
  wait(0.5, seconds);
  SwitchGear(); // Near wall so switch gear for more torque
  minRotSpeed = 100.0;
  pid.turn(-119.0, 3.0); // Slow but precise rotation with enough torque
  pid.setupMove(2.6, Kim, Kdm);
  wait(0.3, seconds);
  SwitchGear(); // Back to 2:1
  pid.move(1050.0, -1.0, 100.0, -170.0, 100.0);
  minRotSpeed = 80.0;
  pid.turn(-200.0, 1.5);
  pid.move(1500.0, -1.0, -1.0, -190.0, -100.0);
  PurpleStorage();
  wait(1, seconds);
  pid.move(100.0, -1.0, 10.0, -180.0, 100.0);
  pid.move(150.0, -1.0, 10.0, -190.0, -100.0);
  wait(0.8, seconds);
  // Goal 2:
  pid.setupMove(5.0, Kim, Kdm);
  pid.move(200.0, -1.0, 100.0, -200.0, 100.0);
  wait(0.5, seconds);
  minRotSpeed = 100.0;
  SwitchGear();
  pid.turn(-208.0);
  wait(0.5, seconds);
  SwitchGear();
  pid.setupMove(3.0, Kim, Kdm);
  pid.move(600.0, -1.0, 300.0, -208.0, 100.0);
  pid.move(450.0, -1.0, 300.0, -230.0, 100.0);
  minRotSpeed = 70.0;
  pid.setupRotate(1.7, 0.0, 0.04);
  pid.turn(-285.0, 1.0);
  pid.move(105.0, -1.0, 100.0, -285.0, 100.0);
  pid.setupMove(2.0, Kim, Kdm);
  pid.move(1000.0, -1.0, 300.0, -285.0, -100.0);
  GreenStorage(1);
  wait(1.75, seconds);
  // Elevator.setMaxTorque(0, percent);
  // Elevator.stop();
  RunIntake();
  // Goal 3:
  pid.setupMove(5.0, Kim, Kdm);
  pid.move(600.0, -1.0, -200.0, -290.0, 100.0);
  pid.move(700.0, -1.0, -200.0, -300.0, 100.0);
  pid.move(500.0, -1.0, -200.0, -310.0, 100.0);
  pid.setupMove(3.0, Kim, Kdm);
  minRotSpeed = 60.0;
  pid.turn(-370.0);
  pid.move(1500.0, -1.0, -1.0, -370.0, -100.0);
  GreenStorage(2);
  wait(1.75, seconds);
  // Full Park (bullshit)
  pid.move(900, -1.0, -1.0, -370.0, 100.0);
}
