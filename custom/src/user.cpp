#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"




// Modify autonomous, driver, or pre-auton code below



void runAutonomous() {
  int auton_selected = 7;
  switch(auton_selected) {
    case 1:
      autonskills();
      break;
    case 2:
      leftandmid();
      break;
    case 3:
      left_right();
      break;
    case 4:
      movetwoinch();
      break;
    case 5:
      rightsidepush();
      break;
    case 6:
      SAWP();
      break; 
    case 7:
      exampleAuton();
      break;
    case 8:
      break;
  }
}




// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;




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


      //descore logic
      static bool descore_pressed = false;
      static bool descore_state = false;
   
   
      if(l1 && !descore_pressed){
        descore_state=!descore_state;
        descore.set(descore_state);
      }
      descore_pressed = l1;
   
      //midgoal logic
      static bool midgoal_pressed = false;
      static bool midgoal_state=false;
   
      if (button_b && !midgoal_pressed){
        //midgoal.set(false);
        midgoal_state=!midgoal_state;
        midgoal.set(midgoal_state);
      }
      midgoal_pressed = button_b;
   
   
   
   
   
   
   
      //scraper logic
      static bool scraper_pressed = false;
      static bool scraper_state = false;
     
      if (button_down_arrow && !scraper_pressed) {
        scraper_state = !scraper_state;
        scraper.set(scraper_state);
      }
      scraper_pressed = button_down_arrow;
   
   
      wait(10, msec); 
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





