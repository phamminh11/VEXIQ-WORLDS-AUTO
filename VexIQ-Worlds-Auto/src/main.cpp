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
motor LM = motor(PORT1, false);
motor RM = motor(PORT7, true);
motor IntakeMotorA = motor(PORT2, true);
motor IntakeMotorB = motor(PORT8, false);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);

/*vex-vision-config:begin*/
vision Vision4 = vision (PORT4, 50);
/*vex-vision-config:end*/
motor ElevatorMotorA = motor(PORT6, true);
motor ElevatorMotorB = motor(PORT12, false);
motor_group Elevator = motor_group(ElevatorMotorA, ElevatorMotorB);

controller Controller = controller();



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "iq_cpp.h"

// Allows for easier use of the VEX Library
using namespace vex;
double prevTime = 0, curTime = 0;
float minSpeed = 0.0;

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
  runIntake = true;
  while (runIntake){
    if(dem >= 50){
      Intake.spin(reverse);
      wait(0.5, seconds);
      Intake.spin(forward);
      dem = 0;
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

void UpdateTimer(){
  while (true){
    curTime = Brain.Timer.value();
    wait(10, msec);
    prevTime = curTime;
  }
}

class PID{
  public:
  float Kp, Kd, Ki;
  void setup(float p, float i, float d){
    Kp = p; Ki = i; Kd = d;
  }

  void Turn(float heading){
    float prevError = heading-BrainInertial.rotation(degrees);
    int dem = 0, maxWaitTime = 10;
    RM.spin(forward); LM.spin(forward);
    while ( dem < maxWaitTime ){
      float cur_angle = BrainInertial.rotation(degrees);
      float error = heading-cur_angle;
      float output = abs(Kp * error + Kd*(error-prevError)/ (curTime-prevTime) + Ki*(error-prevError)*(curTime-prevTime)) * 0.5;
      float vl, vr;
      if(cur_angle < heading)
        vl = output, vr = -output;
      else
        vl = -output, vr = output;
      if(vr < minSpeed && vr > 0.0) vr = minSpeed; //Make sure the robot is still moving very slowly
      if(vl < minSpeed && vl > 0.0) vl = minSpeed;
      if(vr > -minSpeed && vr < 0.0) vr = -minSpeed;
      if(vl > -minSpeed && vl < 0.0) vl = -minSpeed;
      LM.setVelocity(vl, percent); RM.setVelocity(vr, percent);
      if( abs(cur_angle - heading) <= 2.0)
        dem+=1;
      else
        dem=0;
      wait(10,msec);
      prevError = error;
    }
    RM.stop(); LM.stop();
  }

  void Move(float dist, float brake_dist, float heading, float speed){
    float prevError = heading-BrainInertial.rotation(degrees);
    float prevTickL,prevTickR;
    LM.setPosition(0, degrees);   RM.setPosition(0, degrees);
    int dem = 0,    maxWaitTime = 20;
    RM.spin(forward), LM.spin(forward);
    if(speed < 0)
      RM.spin(reverse), LM.spin(reverse);
    while (dist > 0 && dem < maxWaitTime){
      prevTickL = LM.position(turns); 
      prevTickR = RM.position(turns);
      float curGoc = BrainInertial.rotation(degrees);
      float error = heading - curGoc;
      float output = (Kp * error + Kd*(error-prevError)/(curTime-prevTime)) * 0.8;
      float output_speed_ratio = dist/brake_dist;
      if(output_speed_ratio >= 1.0)   output_speed_ratio = 1.0;
      float VR = speed*output_speed_ratio-output,   VL = speed*output_speed_ratio+output;
      if(VR < minSpeed && VR > 0.0) VR = minSpeed; //Make sure the robot is still moving very slowly
      if(VL < minSpeed && VL > 0.0) VL = minSpeed;
      if(VR > -minSpeed && VR < 0.0) VR = -minSpeed;
      if(VL > -minSpeed && VL < 0.0) VL = -minSpeed;
      RM.setVelocity(VR   , percent);
      LM.setVelocity(VL  , percent);
      if( ( abs(LM.velocity(percent)) < 2.0 && abs(RM.velocity(percent)) < 2.0  ) || dist <= 20.0)
        dem+=1;
      else
        dem = 0;
      wait(10, msec);
      prevError = error;
      float moved = abs( (RM.position(turns) - prevTickR + LM.position(turns) - prevTickL) * 199.49 ) * cos((curGoc-heading)/180.0 * 3.1415926 );
      dist -= moved;
    }
    RM.stop(); LM.stop();
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

float Kp = 0.0, Kd = 0.0;
float targetAngle = 0.0;

int main() {
  minSpeed = 5.5;
  Intake.setVelocity(100, percent);
  Elevator.setStopping(hold);
  Elevator.setPosition(0, degrees);
  Elevator.spinToPosition(10, degrees);
  LM.setStopping(brake); RM.setStopping(brake);
  // Begin project code
  thread updatetime = thread(UpdateTimer);
  BrainInertial.calibrate();
  wait(2.5, seconds);
  BrainInertial.setRotation(0, degrees);
  Brain.Screen.print("Calibrated");
  thread printrot = thread(Graphing);
  PID pid; 
  pid.setup(Kp, 0.0, Kd);
  while (true){
    Brain.Screen.print(printToBrain_numberFormat(3), static_cast<float>(Kp));
    Brain.Screen.print("  ");
    Brain.Screen.print(printToBrain_numberFormat(3), static_cast<float>(Kd));
    Brain.Screen.newLine();
    if(Controller.ButtonEUp.pressing()) //KP INCREASE
      Kp += 0.01;
    if(Controller.ButtonEDown.pressing()) //KP DECREASE
      Kp -= 0.01;
    if(Controller.ButtonFUp.pressing()) //KD INCREASE
      Kd += 0.01;
    if(Controller.ButtonFDown.pressing()) //KD DECREASE
      Kd -= 0.01;
    if(Controller.ButtonRUp.pressing()){ //TURN RIGHT 90 DEGREE
      targetAngle += 90.0;
      pid.setup(Kp, 0.0, Kd);
      pid.Turn(targetAngle);
    }
    if(Controller.ButtonLUp.pressing()){ //TURN LEFT 90 DEGREE
      targetAngle -= 90.0;
      pid.setup(Kp, 0.0, Kd);
      pid.Turn(targetAngle);
    }
    wait(0.1, seconds);
  }

  //GyroTurn(90.0, 1.1, 0.8);
  // thread runIntake = thread(RunIntake);
  // StopIntake();
  // Intake.spin(reverse);
  // for(int i = 0; i < 6; i++){
  //   GyroMove(1000.0, 150.0, -7.5, 75.0, 0.5, 0.1);
  //   runIntake = thread(RunIntake);
  //   GyroMove(200.0, 50.0, 0.0, -75.0, 0.6, 0.1);
  //   GyroMove(200.0, 50.0, 0.0, 75.0, 0.6, 0.1);
  //   GyroMove(200.0, 50.0, 0.0, -75.0, 0.6, 0.1);
  //   GyroMove(200.0, 50.0, 0.0, 75.0, 0.6, 0.1);
  //   GyroMove(200.0, 50.0, 0.0, -75.0, 0.6, 0.1);
  //   GyroMove(200.0, 50.0, 0.0, 75.0, 0.6, 0.1);
  //   GyroMove(200.0, 50.0, 0.0, -75.0, 0.5, 0.1);
  //   StopIntake();
  //   wait(0.5, seconds);
  //   Intake.spin(reverse);
  //   GyroTurn(-7.6, 1.3, 0.5);
  // }
}