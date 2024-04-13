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
motor LM = motor(PORT12, true);
motor RM = motor(PORT6, false);
motor IntakeMotorA = motor(PORT1, false);
motor IntakeMotorB = motor(PORT7, true);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);

/*vex-vision-config:begin*/
vision Vision4 = vision (PORT4, 50);
/*vex-vision-config:end*/
motor ElevatorMotorA = motor(PORT8, true);
motor ElevatorMotorB = motor(PORT2, false);
motor_group Elevator = motor_group(ElevatorMotorA, ElevatorMotorB);

controller Controller = controller();

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "iq_cpp.h"

// Allows for easier use of the VEX Library
using namespace vex;
double previousTime = 0, currentTime = 0;
float minSpeed = 0.0;


bool runIntake = false;
void RunIntake(float DELAY)
{
  int count = 0;
  runIntake = true;
  wait(DELAY, seconds);
  while (runIntake)
  {
    if (count >= 50)
    {
      Intake.spin(reverse);
      wait(0.5, seconds);
      Intake.spin(forward);
      count = 0;
    }
    Intake.spin(forward);
    if (Intake.velocity(percent) == 0.0)
      count += 1;
    else
      count = 0;
    wait(5, msec);
  }
  Intake.stop();
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
      integral += Kir * (error - previousError) * (currentTime - previousTime);
      float output = abs(Kpr * error + Kdr * (error - previousError) / (currentTime - previousTime) + integral) * 0.5;
      float leftVelocity, rightVelocity;
      if (currentAngle < heading)
        leftVelocity = output, rightVelocity = -output;
      else
        leftVelocity = -output, rightVelocity = output;
      if (rightVelocity < minSpeed && rightVelocity > 0.5)
        rightVelocity = minSpeed; // Make sure the robot is still moving very slowly
      if (leftVelocity < minSpeed && leftVelocity > 0.5)
        leftVelocity = minSpeed;
      if (rightVelocity > -minSpeed && rightVelocity < -0.5)
        rightVelocity = -minSpeed;
      if (leftVelocity > -minSpeed && leftVelocity < -0.5)
        leftVelocity = -minSpeed;
      LM.setVelocity(leftVelocity, percent);
      RM.setVelocity(rightVelocity, percent);
      if (abs(currentAngle - heading) <= 2.5)
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
    LM.setPosition(0, degrees);
    RM.setPosition(0, degrees);
    int count = 0, maxCount = 18;
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
        if (output_MUL < 0.3)
          output_MUL = 0.3;
      }
      else if (breakeDistance >= distance && distance >= 0.0)
        output_MUL = distance / breakeDistance;
      float rightVelocity = speed * output_MUL - output, leftVelocity = speed * output_MUL + output;
      if (rightVelocity < 45.0 && rightVelocity >= 0.0)
        rightVelocity = 45.0; // Make sure the robot is still moving very slowly
      if (leftVelocity < 45.0 && leftVelocity >= 0.0)
        leftVelocity = 45.0;
      if (rightVelocity > -45.0 && rightVelocity < 0.0)
        rightVelocity = -45.0;
      if (leftVelocity > -45.0 && leftVelocity < 0.0)
        leftVelocity = -45.0;
      RM.setVelocity(rightVelocity, percent);
      LM.setVelocity(leftVelocity, percent);
      if ((abs(LM.velocity(percent)) == 0.0 && abs(RM.velocity(percent)) == 0.0))
        count += 1;
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

void graphing()
{
  while (true)
  {
    double t = Brain.Timer.value();
    printf("%.3f", t);
    printf("    ");
    printf("%.3f", BrainInertial.rotation(degrees));
    printf("\n");
    wait(0.1, seconds);
  }
}

float Kpr = 0.8, Kdr = 0.1, Kir = 0.0; //Default PID rotation value
float Kpm = 1.8, Kdm = 0.12, Kim = 0.0; //Defailt PID movement value
float defaultAccelDistance = 500.0;
float defaultBrakeDistance = 200.0;
PID pid;

void setup()
{
  minSpeed = 7.0;
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
  thread printrot = thread(graphing);
  pid.setupRotate(Kpr, Kir, Kdr);
  pid.setupMove(Kpm, Kim, Kdm);
}

void clearSupplyZone()
{
  defaultAccelDistance = 500.0;
  defaultBrakeDistance = 200.0;
  thread intake_t = thread([]{ RunIntake(0); });
  pid.move(1500.0, defaultAccelDistance, defaultBrakeDistance, 0.0, 60.0);
  wait(0.1, seconds);
  runIntake = false;
  pid.turn(110.0);
  Intake.spin(reverse);
  // pid.setupRotate(4.0, Kdr, Kir);
  defaultBrakeDistance = 0.0;
  for (int i = 0; i < 4; i++)
  {
    pid.move(1000.0, defaultAccelDistance, defaultBrakeDistance, 90.0, 80.0);
    if (i == 0)
      intake_t = thread([]
                        { RunIntake(1.0); });
    pid.move(300.0, defaultAccelDistance, defaultBrakeDistance, 90.0, -60.0);
    pid.move(300.0, defaultAccelDistance, defaultBrakeDistance, 90.0, 80.0);
    pid.move(300.0, defaultAccelDistance, defaultBrakeDistance, 90.0, -60.0);
    pid.move(300.0, defaultAccelDistance, defaultBrakeDistance, 90.0, 80.0);
    pid.move(200.0, defaultAccelDistance, defaultBrakeDistance, 90.0, -60.0);
    // pid.turn(60.0);
    LM.spin(forward);
    RM.spin(forward);
    LM.setVelocity(-50.0, percent);
    RM.setVelocity(50.0, percent);
    wait(0.72, seconds);
    LM.stop();
    RM.stop();
    wait(0.2, seconds);
  }
  runIntake = false;
}

int main()
{
  setup();
  //Autonomous path
  pid.move(600.0, 10.0, 300.0, 0.0, 80.0);
  wait(0.1, seconds);
  pid.turn(-90.0);
  wait(0.1, seconds);
  pid.move(600.0, 10.0, 300.0, -90.0, 80.0);
  wait(0.1, seconds);
  pid.turn(0.0);
  //wait(0.25, seconds);
  pid.move(50.0, -1.0, 50.0, 0.0, -80.0);
  //wait(0.2, seconds);
  pid.move(700.0, 10.0, 100.0, 0.0, 80.0);
  //wait(0.1, seconds);
  pid.turn(35.0);
  wait(0.1,seconds);
  pid.move(230.0, 5.0, 50.0, 35.0, 40.0);
  //wait(0.2, seconds); 
  pid.move(230.0, 5.0, 50.0, 35.0, -40.0);
  wait(0.1, seconds);
  pid.turn(0.0);
  //wait(0.1, seconds);
  pid.setupMove(1.9, Kim, Kdm);
  pid.move(1000.0, -1.0, 120.0, -90.0, 80.0);
  pid.move(300.0, 10.0, 100.0, -90.0, -60.0);
  //wait(0.1, seconds);
  pid.turn(-135.0);
  //wait(0.1, seconds);
  pid.setupMove(2.0, Kim, Kdm);
  pid.move(1200.0, -1.0, 400.0, -180.0, 80.0);
  wait(0.1, seconds);
  pid.turn(-60.0);
  //wait(0.1, seconds);
  pid.move(450.0, -1.0, 200.0, 10.0, 75.0);
  pid.move(3000.0, -1.0, 400.0, 0.0, -100.0);
}