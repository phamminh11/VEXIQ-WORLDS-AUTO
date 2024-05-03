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
touchled TouchLED = touchled(PORT5);

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
    Pneumatic4.extend(cylinder1);
    Pneumatic5.extend(cylinder1);
  }
  else
  {
    two_to_one = false;
    Pneumatic4.retract(cylinder1);
    Pneumatic5.retract(cylinder1);
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
    int count = 0, maxCount = 8;
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

float Kpr = 1.0, Kdr = 0.05, Kir = 0.0; // Default PID rotation value
float Kpm = 10.0, Kdm = 0.0, Kim = 0.0; // Defailt PID movement value
float defaultAccelDistance = 500.0;
float defaultBrakeDistance = 200.0;
PID pid;

void clearSupplyZone()
{
  pid.setupMove(3.0, Kim, Kdm);
  pid.move(450.0, -1.0, 100.0, 90.0, 100.0);
  pid.move(50.0, -1.0, 10.0, 90.0, -100.0);
  wait(0.8, seconds);
  LM.spin(forward);
  RM.spin(forward);
  LM.setVelocity(-100, percent);
  RM.setVelocity(100, percent);
  wait(0.4, seconds);
  LM.setVelocity(100, percent);
  RM.setVelocity(-100, percent);
  wait(0.4, seconds);
  LM.stop();
  RM.stop();
  for (int i = 0; i < 2; i++)
  {
    wait(0.8, seconds);
    pid.move(50.0, -1.0, 10.0, 90.0, -40.0);
    pid.move(150.0, -1.0, 10.0, 90.0, 40.0);
    if (i == 2)
      continue;
    LM.spin(forward);
    RM.spin(forward);
    LM.setVelocity(100, percent);
    RM.setVelocity(-100, percent);
    wait(0.4, seconds);
    LM.setVelocity(-100, percent);
    RM.setVelocity(100, percent);
    wait(0.4, seconds);
    LM.stop();
    RM.stop();
  }
  pid.move(180.0, -1.0, 100.0, 90.0, -100.0);
  wait(0.5, seconds);
  minRotSpeed = 70.0;
  pid.turn(138.0, 1.3);
  pid.setupMove(1.0, Kim, Kdm);
  pid.move(100.0, -1.0, 100.0, 90.0, 50.0);
  for (int i = 0; i < 2; i++)
  {
    wait(0.75, seconds);
    pid.move(50.0, -1.0, 10.0, 90.0, -40.0);
    pid.move(150.0, -1.0, 10.0, 90.0, 40.0);
    if (i == 2)
      continue;
    LM.spin(forward);
    RM.spin(forward);
    LM.setVelocity(100, percent);
    RM.setVelocity(-100, percent);
    wait(0.4, seconds);
    LM.setVelocity(-100, percent);
    RM.setVelocity(100, percent);
    wait(0.4, seconds);
    LM.stop();
    RM.stop();
  }
  minRotSpeed = 50.0;
  pid.turn(90.0, 1.0);
  pid.move(430.0, -1.0, 200.0, 90.0, -100.0);
  // wait(1000, seconds);
}

void setup()
{
  minRotSpeed = 10.0;
  minMoveSpeed = 30.0;
  Pneumatic4.retract(cylinder1);
  Pneumatic5.retract(cylinder1);
  Pneumatic4.extend(cylinder2);
  Pneumatic5.retract(cylinder2);
  Intake.setVelocity(100, percent);
  Elevator.setVelocity(100, percent);
  Elevator.setStopping(hold);
  Elevator.setPosition(0, turns);
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
  TouchLED.setColor(orange);

  while (!TouchLED.pressing())
  {
    wait(20, msec);
  }

  TouchLED.setColor(violet);
  setup();
  // Autonomous path
  RunIntake();
  minRotSpeed = 15.0;
  pid.move(1150.0, -10.0, -10.0, 7.0, 100.0);
  pid.turn(75.0, 2.0);
  pid.move(80.0, -10.0, 50.0, 80.0, -50.0);
  // Supply zone phase
  clearSupplyZone(); // Need improvement for more green
  // Goal 1
  minRotSpeed = 15.0;
  pid.turn(-30.0, 2.5);
  pid.setupMove(1.7, Kim, Kdm);
  wait(0.3, seconds);
  // pid.move(100.0, -1.0, -200.0, -45.0, 60.0);
  pid.move(450.0, -1.0, 100.0, -70.0, 60.0); // Stopping too much
  wait(0.7, seconds);
  // pid.move(180.0, -1.0, -100.0, -85.0, 100.0);
  // wait(0.4, seconds);
  pid.move(530.0, -1.0, 100.0, -85.0, 60.0);
  // pid.setupMove(5.0, Kim, Kdm);
  // pid.move(250.0, -1.0, 10.0, -75.0, -100.0);
  wait(0.3, seconds);
  SwitchGear();        // Near wall: switch gear for more torque
  minRotSpeed = 100.0; // Slow but precise rotation with 4x torque
  pid.turn(-130.0, 3.0);
  wait(0.3, seconds);
  SwitchGear(); // Back to 2:1
  // wait(1000, seconds);
  pid.setupMove(5.0, Kim, Kdm);
  pid.move(200.0, 50.0, -1.0, -130.0, 100.0);
  pid.setupMove(2.7, Kim, Kdm);
  // pid.move(50.0, -1.0, -100.0, -130.0, 170);
  LM.spin(forward);
  RM.spin(forward);
  LM.setVelocity(100.0, percent);
  RM.setVelocity(-100.0, percent);
  wait(0.28, seconds);
  // LM.spin(forward); RM.spin(forward);
  LM.setVelocity(-100.0, percent);
  RM.setVelocity(+100.0, percent);
  wait(0.3, seconds);
  pid.move(600.0, -1.0, -100.0, -160.0, 100.0);
  pid.move(250.0, -1.0, 100.0, -170.0, 50.0);
  minRotSpeed = 80.0;
  LM.spin(forward);
  RM.spin(forward);
  LM.setVelocity(-100.0, percent);
  RM.setVelocity(100.0, percent);
  wait(0.44, seconds);
  LM.stop();
  RM.stop();
  pid.setupMove(2.0, Kim, Kdm);
  thread dropPurple = thread([]
                             {wait(1.72, seconds); PurpleStorage(); });
  pid.move(350.0, -1.0, -1.0, -190.0, -100.0);
  pid.move(1000.0, -1.0, -1.0, -190.0, -60.0);
  wait(0.7, seconds);
  // pid.move(80.0, -1.0, 10.0, -180.0, 100.0);
  // pid.move(150.0, -1.0, 10.0, -190.0, -100.0);
  // Goal 2:
  pid.setupMove(5.0, Kim, Kdm);
  pid.move(200.0, -1.0, 10.0, -200.0, 100.0);
  wait(0.3, seconds);
  minRotSpeed = 100.0;
  SwitchGear();
  pid.turn(-206.0);
  wait(0.3, seconds);
  SwitchGear();
  PurpleStorage();
  pid.setupMove(3.0, Kim, Kdm);
  pid.move(600.0, -1.0, -300.0, -206.0, 100.0);
  pid.move(550.0, -1.0, 300.0, -233.0, 100.0);
  minRotSpeed = 70.0;
  pid.setupRotate(2.0, 0.0, 0.04);
  RunIntake();
  pid.turn(-280.0, 1.0);
  pid.move(150.0, -1.0, -10.0, -290.0, 40.0);
  pid.setupMove(2.0, Kim, Kdm);
  thread pour_green = thread([]{ Elevator.setPosition(0, turns); wait(0.5, seconds); Elevator.setVelocity(100, percent); while(Elevator.position(turns) > -1.54){ Elevator.spin(reverse); wait(5, msec);} TouchLED.setColor(red); Elevator.stop(); });
  pid.move(1000.0, -1.0, 300.0, -285.0, -100.0);
  // Elevator.setVelocity(100, percent);
  // Elevator.setPosition(0, turns);
  // Elevator.spinToPosition(1.5, turns);
  // Green 1
  pid.move(80.0, -1.0, 10.0, -270.0, 100.0);
  pid.move(100.0, -1.0, 10.0, -280.0, -100.0);
  wait(0.5, seconds);
  RunIntake();
  // Goal 3:
  pid.setupMove(5.0, Kim, Kdm);
  pid.move(600.0, -1.0, -200.0, -290.0, 100.0);
  pid.move(600.0, -1.0, -200.0, -300.0, 100.0);
  pid.move(650.0, -1.0, 200.0, -300.0, 100.0);
  pid.setupMove(1.5, Kim, Kdm);
  pid.setupRotate(1.8, Kir, Kdr);
  minRotSpeed = 30.0;
  pid.turn(-380.0, 3.0);
  wait(0.2, seconds);
  RunIntake();
  // Green ams
  thread pour_green2 = thread([]{Elevator.spin(reverse); wait(1.9, seconds); Elevator.stop();});
  
  LM.setStopping(coast);
  RM.setStopping(coast);
  pid.move(1500.0, -1.0, -1.0, -365.0, -100.0);
  wait(1.9, seconds);
  RunIntake();
  LM.spin(forward);
  RM.spin(forward);
  // Park
  pid.move(900.0, -1.0, -1.0, -360.0, 100.0);
}
