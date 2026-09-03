#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"




// Modify autonomous, driver, or pre-auton code below



void runAutonomous() {
  int auton_selected = 3;
  switch(auton_selected) {
    case 1:
      SAWP();
      break;
    case 2:
      RedRight();
      break;
    case 3:
      BlueLeft();
      break;
    case 4:
      movetwoinch();
      break;
    case 5:
      exampleAuton();
      break;
    case 6:
      BlueLeftAutonTwo(); 
      break;
    case 7:
      break;
  }
}




// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;


bool already_up = false;
bool claw_zeroed = false;   // edge-trigger so the claw only auto-homes once per return to bottom

const double CASCADE_HOME_TOL = 5.0; // degrees of slop around 0


void runDriver() {
  //r1 and r2
   // Set brake mode to coast at the start of driver control
  stopChassis(coast);
  heading_correction = false;

  while (true) {

    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);

      // true/false for controller button presses
      l1 = controller_1.ButtonL1.pressing();
      l2 = controller_1.ButtonL2.pressing();
      r1 = controller_1.ButtonR1.pressing();
      r2 = controller_1.ButtonR2.pressing();
      button_a = controller_1.ButtonA.pressing();
      button_b = controller_1.ButtonB.pressing();
      button_x = controller_1.ButtonX.pressing();
      button_y = controller_1.ButtonY.pressing();
      button_up_arrow = controller_1.ButtonUp.pressing();
      button_down_arrow = controller_1.ButtonDown.pressing();
      button_left_arrow = controller_1.ButtonLeft.pressing();
      button_right_arrow = controller_1.ButtonRight.pressing();

      // 1. Get Joystick Inputs (VEXcode scale is actually -100 to 100, but .value() maxes at 127/100 depending on setup. Assuming max 127 here)
      // Left stick Vertical = Forward, Right stick Horizontal = Turn
      double forwardInput = (double)controller_1.Axis3.value(); 
      double turnInput = (double)controller_1.Axis1.value(); // Norrel was here

    // 2. Normalize to [-1.0, 1.0] for cleaner math
      double f = forwardInput / 127.0;
      double t = turnInput / 127.0;

     // === TUNING PARAMETERS ===
    const double QUICK_TURN_GAIN = 0.8; // High sensitivity for spinning in place (1.0 is default)
    const double CURVE_GAIN = 1.6;      // Amplifies the power difference between left and right sides during a curve 
    
    // 3. Curvature Drive Logic
    // We want the turn gripping power to change based on forward speed.
    // If stopped, we still want to be able to quick-turn (arcade style).
    
    
    double leftOutputScaled, rightOutputScaled;

    if (fabs(f) < 0.05) {
        // Quick-turn logic: If not moving forward, do a standard spin-in-place
        leftOutputScaled = t * QUICK_TURN_GAIN;
        rightOutputScaled = -t * QUICK_TURN_GAIN;
    } else {
    // Curvature logic: Turn input is multiplied by forward velocity
    // This creates a constant turning radius
        double turnModifier = t * fabs(f) * CURVE_GAIN;
    
        leftOutputScaled = f + turnModifier;
        rightOutputScaled = f - turnModifier;

        // Normalize the outputs if they exceed 1.0 to preserve the turning ratio
       double maxOutput = fmax(fabs(leftOutputScaled), fabs(rightOutputScaled));
       if (maxOutput > 1.0) {
           leftOutputScaled /= maxOutput;
           rightOutputScaled /= maxOutput;
       }
    }

    // 4. Convert back to motor voltage units (-12V to 12V, see driveChassis)
    double leftOutput = leftOutputScaled * 12.0;
    double rightOutput = rightOutputScaled * 12.0;

    // 4. Move the Chassis
    driveChassis(leftOutput,rightOutput);
  
    //y is intake piston up
    if (button_y){
      if (already_up == false){
        intake_pnuematics.set(true);
        already_up = true;
      }else if (already_up == true || intake_sensor.objectDistance(mm) < 135 ){ //has intake code
        intake_pnuematics.set(false);
        already_up = false;
      }
    }

    /*intake code
    if (intake_sensor.objectDistance(mm) < 135){ //tune value of 260 to whatever the actual number is. 
      intake1.set(false);
      intake2.set(false);
      already_up = false;
    }
      */

    //cascade code
    if (l1){
      //cascade up
      cascade.spin(fwd, 12, volt);
      claw.spinToPosition(90, degrees, false);
    }else if(l2){
      //cascade down
      cascade.spin(reverse, 12, volt);
    }else{
      cascade.spin(fwd, 0, volt);
    }

    if (fabs(cascade.position(degrees)) < CASCADE_HOME_TOL) {
      if (!claw_zeroed) {
        claw.spinToPosition(0, degrees, false); // non-blocking so the drive loop keeps running
        claw_zeroed = true;
      }
    } else {
      claw_zeroed = false; // re-arm once the cascade has left the bottom
    }

    
    //claw
   if (button_a){
      if (already_90 == false){
        claw.spinToPosition(90, degrees, false);
        already_90 = true;
      }else if (already_90 == true){
        claw.spinToPosition(0, degrees, false);
        already_90 = false;
      }
    }

    //intake code logic
    if(r1){
      //intake
      intake.spin(fwd, 12,volt);
      claw_intake.spin(fwd, 12,volt);
    } else if(r2){
      //outake
      intake.spin(reverse,12,volt);
      claw_intake.spin(reverse,12,volt);
      claw.spinToPosition(125, degrees, false);
      wait(1000, msec);
    } else {
      intake.spin(fwd,0,volt);
      claw_intake.spin(fwd,0,volt);
    }

  }

  
}




void runPreAutonomous() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }
 
  
  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);
  // odom tracking
  resetChassis();

  vertical_tracker.resetPosition();

  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }
}





