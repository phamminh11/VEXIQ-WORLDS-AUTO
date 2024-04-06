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

class PID{
  public:
  float Kpr, Kdr, Kir;
  float Kpm, Kdm, Kim;
  void setup_rotate(float p, float i, float d){
    Kpr = p; Kir = i; Kdr = d;
  }
  void setup_move(float p, float i, float d){
    Kpm = p; Kim = i; Kdm = d;
  }

  void Turn(float HEADING){
    float PREV_ERROR = HEADING-BrainInertial.rotation(degrees);
    float INTEGRAL = 0.0;
    int COUNT = 0, MAXCOUNT = 10;
    RM.spin(forward); LM.spin(forward);
    while ( COUNT < MAXCOUNT ){
      float cur_angle = BrainInertial.rotation(degrees);
      float ERROR = HEADING-cur_angle;
      INTEGRAL += Kir*(ERROR-PREV_ERROR)*(CURRENT_TIME-PREV_TIME);
      float OUTPUT = abs(Kpr * ERROR + Kdr*(ERROR-PREV_ERROR)/ (CURRENT_TIME-PREV_TIME) + INTEGRAL) * 0.5;
      float VL, VR;
      if(cur_angle < HEADING)
        VL = OUTPUT, VR = -OUTPUT;
      else
        VL = -OUTPUT, VR = OUTPUT;
      if(VR < MINSPEED && VR > 0.0) VR = MINSPEED; //Make sure the robot is still moving very slowly
      if(VL < MINSPEED && VL > 0.0) VL = MINSPEED;
      if(VR > -MINSPEED && VR < 0.0) VR = -MINSPEED;
      if(VL > -MINSPEED && VL < 0.0) VL = -MINSPEED;
      LM.setVelocity(VL, percent); RM.setVelocity(VR, percent);
      if( abs(cur_angle - HEADING) <= 2.0)
        COUNT+=1;
      else
        COUNT=0;
      wait(10,msec);
      PREV_ERROR = ERROR;
    }
    RM.stop(); LM.stop();
  }

  void Move(float DIST, float ACCEL_DIST, float BRAKE_DIST, float HEADING, float SPEED){
    float PREV_ERROR = HEADING-BrainInertial.rotation(degrees);
    float PREV_TICK_L,PREV_TICK_R;
    float ORIGINAL_DIST = DIST;
    LM.setPosition(0, degrees);   RM.setPosition(0, degrees);
    int COUNT = 0,    MAXCOUNT = 15;
    RM.spin(forward), LM.spin(forward);
    if(SPEED < 0){
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("reverse");
      RM.spin(reverse); LM.spin(reverse);
    }
    while (DIST > 0 && COUNT < MAXCOUNT){
      PREV_TICK_L = LM.position(turns); 
      PREV_TICK_R = RM.position(turns);
      float ANGLE = BrainInertial.rotation(degrees);
      float ERROR = HEADING - ANGLE;
      float OUTPUT = (Kpm * ERROR + Kdm*(ERROR-PREV_ERROR)/(CURRENT_TIME-PREV_TIME)) * 0.5;
      float OUTPUT_MUL = 1.0;
      if(ORIGINAL_DIST > DIST > ORIGINAL_DIST-ACCEL_DIST){
        OUTPUT_MUL = (ORIGINAL_DIST-DIST)/ACCEL_DIST;
        if(OUTPUT_MUL > 0.7) OUTPUT_MUL = 0.7;
      }
      else if(BRAKE_DIST > DIST > 0.0)
        OUTPUT_MUL = DIST/BRAKE_DIST;
      float VR = SPEED*OUTPUT_MUL-OUTPUT,   VL = SPEED*OUTPUT_MUL+OUTPUT;
      if(VR < MINSPEED && VR > 0.0) VR = MINSPEED; //Make sure the robot is still moving very slowly
      if(VL < MINSPEED && VL > 0.0) VL = MINSPEED;
      if(VR > -MINSPEED && VR < 0.0) VR = -MINSPEED;
      if(VL > -MINSPEED && VL < 0.0) VL = -MINSPEED;
      RM.setVelocity(VR, percent);
      LM.setVelocity(VL, percent);
      if( ( abs(LM.velocity(percent)) < 2.0 && abs(RM.velocity(percent)) < 2.0  ) || DIST <= 20.0)
        COUNT+=1;
      else
        COUNT = 0;
      //Brain.Screen.print("%.5f", static_cast<float>(DIST));
      wait(10, msec);
      PREV_ERROR = ERROR;
      float MOVED = abs( (RM.position(turns) - PREV_TICK_R + LM.position(turns) - PREV_TICK_L) * 199.49 ) * cos((ANGLE-HEADING)/180.0 * 3.1415926 );
      DIST -= MOVED;
      //Brain.Screen.clearScreen();
      //Brain.Screen.setCursor(1, 1);
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

float Kpr = 1.8, Kdr = 0.15, Kir = 0.0;
float Kpm = 1.8, Kdm  = 0.15, Kim = 0.0;
float targetAngle = 0.0;

int main() {
  MINSPEED = 10.0;
  Intake.setVelocity(100, percent);
  Elevator.setStopping(hold);
  Elevator.setPosition(0, degrees);
  Elevator.spinToPosition(10, degrees);
  LM.setStopping(coast); RM.setStopping(coast);
  // Begin project code
  thread updatetime = thread(UpdateTimer);
  BrainInertial.calibrate();
  wait(2.5, seconds);
  BrainInertial.setRotation(0, degrees);
  Brain.Screen.print("Calibrated");
  thread printrot = thread(Graphing);

  PID pid; 
  pid.setup_rotate(Kpr, Kir, Kdr);
  pid.setup_move(Kpm, Kim, Kdm);
  //LM.spin(forward); RM.spin(forward);
  while (1){
    Brain.Screen.print(printToBrain_numberFormat(2), static_cast<float>(MINSPEED));
    Brain.Screen.newLine();
    //LM.setVelocity(MINSPEED, percent); RM.setVelocity(-MINSPEED, percent);
    Brain.Screen.print(printToBrain_numberFormat(2), static_cast<float>(Kpr));
    Brain.Screen.print("  ");
    Brain.Screen.print(printToBrain_numberFormat(2), static_cast<float>(Kdr));
    Brain.Screen.print("  ");
    Brain.Screen.print(printToBrain_numberFormat(2), static_cast<float>(Kir));
    Brain.Screen.newLine();
    // if(Controller.AxisA.position() > 20.0){
    //   MINSPEED += 0.02;
    // }
    // if(Controller.AxisA.position() < -20.0){
    //   MINSPEED -= 0.02;
    // }
    if(Controller.ButtonEUp.pressing()) //KP INCREASE
      Kpr += 0.01;
    if(Controller.ButtonEDown.pressing()) //KP DECREASE
      Kpr -= 0.01;
    if(Controller.ButtonFUp.pressing()) //KD INCREASE
      Kdr += 0.01;
    if(Controller.ButtonFDown.pressing()) //KD DECREASE
      Kdr -= 0.01;
    if(Controller.AxisD.position() > 20.0)
      Kir += 0.05;
    if(Controller.AxisD.position() < -20.0)
      Kir -= 0.05;
  
    if(Controller.ButtonRUp.pressing()){ //TURN RIGHT 90 DEGREE
      targetAngle += 90.0;
      pid.setup_rotate(Kpr, Kir, Kdr);
      Brain.Screen.print("test");
      pid.Turn(targetAngle);
    }
    if(Controller.ButtonLUp.pressing()){ //TURN LEFT 90 DEGREE
      targetAngle -= 90.0;
      pid.setup_rotate(Kpr, Kir, Kdr);
      pid.Turn(targetAngle);
    }
    if(Controller.ButtonRDown.pressing()){
      pid.setup_move(Kpr, Kir, Kdr);
      pid.Move(300.0, 100.0, 150.0, 0.0, 60.0);
    }
    if(Controller.ButtonLDown.pressing()){
      pid.setup_move(Kpr, Kir, Kdr);
      pid.Move(300.0, 100.0, 150.0, 0.0, -60.0);
    }
    wait(0.05, seconds);
  }
}