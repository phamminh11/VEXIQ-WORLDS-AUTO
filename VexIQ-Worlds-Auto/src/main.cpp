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
motor IntakeMotorA = motor(PORT3, false);
motor IntakeMotorB = motor(PORT9, true);
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
      if( abs(cur_angle - HEADING) <= 2.5)
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
    int COUNT = 0,    MAXCOUNT = 18;
    RM.spin(forward), LM.spin(forward);
    if(SPEED < 0)
      RM.spin(reverse), LM.spin(reverse);
    while (DIST > 0 && COUNT < MAXCOUNT){
      PREV_TICK_L = LM.position(turns); 
      PREV_TICK_R = RM.position(turns);
      float curGoc = BrainInertial.rotation(degrees);
      float ERROR = HEADING - curGoc;
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
      if( ( abs(LM.velocity(percent)) == 0.0 && abs(RM.velocity(percent)) == 0.0  ) || DIST <= 20.0)
        COUNT+=1;
      else
        COUNT = 0;
      wait(10, msec);
      PREV_ERROR = ERROR;
      float MOVED = abs( (RM.position(turns) - PREV_TICK_R + LM.position(turns) - PREV_TICK_L) * 199.49 ) * cos((curGoc-HEADING)/180.0 * 3.1415926 );
      DIST -= MOVED;
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

float Kpr = 1.05, Kdr = 0.0, Kir = 40.1;
float Kpm = 2.8, Kdm  = 0.12, Kim = 0.0;
float targetAngle = 0.0;

int main() {
  MINSPEED = 13.0;
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
  pid.setup_rotate(Kpr, Kir, Kdr);
  pid.setup_move(Kpm, Kim, Kdm);
  
  thread intake_t = thread([]{RunIntake(0);});
  pid.Move(1500.0, 500.0, 150.0, 0.0, 60.0);
  wait(0.1, seconds);
  runIntake = false;
  pid.Turn(110.0);
  Intake.spin(reverse);
  //pid.setup_rotate(4.0, Kdr, Kir);
  for(int i = 0; i < 4; i++){
    pid.Move(1000.0, 0.0, 150.0, 90.0, 80.0);
    if(i == 0)
      intake_t = thread([]{RunIntake(1.0);});
    pid.Move(300.0, 0.0, 150.0, 90.0, -60.0);
    pid.Move(300.0, 0.0, 150.0, 90.0, 80.0);
    pid.Move(300.0, 0.0, 150.0, 90.0, -60.0);
    pid.Move(300.0, 0.0, 150.0, 90.0, 80.0);
    pid.Move(200.0, 0.0, 150.0, 90.0, -60.0);
    //pid.Turn(60.0);
    LM.spin(forward); RM.spin(forward);
    LM.setVelocity(-50.0, percent); RM.setVelocity(50.0, percent);
    wait(0.72, seconds);
    LM.stop(); RM.stop(); 
    wait(0.2, seconds);
  }
  runIntake = false;
}