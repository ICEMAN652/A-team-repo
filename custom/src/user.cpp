#include "vex.h"
#include "motor-control.h"
#include "auton-selector.h"
#include "../custom/include/autonomous.h"




// Modify autonomous, driver, or pre-auton code below



void runAutonomous() {
  // Runs the routine picked on the Brain screen (or with the controller arrows while disabled).
  // Add routines to the list in runPreAutonomous() below.
  runSelectedAuton();
}




// controller_1 input variables (snake_case)
//don't touch
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;
bool claw_toggle = false;
bool r1_last_state = false;
bool claw_up = false;
bool r2_last_state = false;
bool b_last_state = false;
bool toggle_b_moving = false; // true while B's 360 degree turn is running
bool claw_reversing = false; // latched on by R2, off by the next R1/R2 press
bool claw_idle_spin = false; // true after R1 intake -> claw holds at 2V; cleared by R2
bool cascade_auto_moving = false;
bool lift_homing = false; // true while Y is driving the lift down to 0
//const double LIFT_MAX_DEG = 975.0; // top limit for L1, in motor degrees (360 = 1 rotation). 


bool already_up = false;
bool r1_prev = false;
//bool claw_zeroed = false;   // edge-trigger so the claw only auto-homes once per return to bottom

//const double CASCADE_HOME_TOL = 15.0; // degrees of slop around 0 


void runDriver() {
  //r1 and r2
   // Set brake mode to coast at the start of driver control
  stopChassis(coast);
  heading_correction = false;

  

  while (true) {
    
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("Lift: %.0f deg     ", lift.position(degrees));
    // lift motor diagnostics: connected?, temp (C), current (A), own encoder (deg)
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("P12: %s %.0fC %.1fA %.0fdeg   ", lift_1.installed() ? "OK" : "DC",
                       lift_1.temperature(celsius), lift_1.current(amp), lift_1.position(degrees));
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("P13: %s %.0fC %.1fA %.0fdeg   ", lift_2.installed() ? "OK" : "DC",
                       lift_2.temperature(celsius), lift_2.current(amp), lift_2.position(degrees));

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
    

    if (l1) {
      lift_homing = false; // manual input cancels the Y auto-down
      lift.spin(forward, 100, velocityUnits::pct);
    }else if (l2) {
      lift_homing = false;
      lift.spin(reverse, 100, velocityUnits::pct);
    }
    else if (button_y && !lift_homing){
      // start the move once; it keeps running after Y is released
      lift_homing = true;
      lift.setVelocity(50, velocityUnits::pct);
      lift.spinToPosition(0, degrees, false);
    }
    else if (lift_homing){
      // let spinToPosition finish, then fall through to hold
      if (fabs(lift.position(degrees)) < 5) {
        lift_homing = false;
        lift.stop(hold);
      }
    }
    else{
      lift.stop(hold);
    }
  

    // edge detect so a single press toggles, not every loop while held
    bool r1_pressed = r1 && !r1_last_state;
    bool r2_pressed = r2 && !r2_last_state;
    r1_last_state = r1;
    r2_last_state = r2;

    if (claw_reversing) {
      // R2 latches reverse on; the next R1 or R2 press turns it off
      if (r1_pressed || r2_pressed) {
        claw_reversing = false;
      }
    } else if (r2_pressed) {
      claw_reversing = true;
    }

    // R1 arms the 2V idle spin (after intaking); R2 disarms it until R1 is pressed again
    if (r1) {
      claw_idle_spin = true;
    }
    if (r2_pressed) {
      claw_idle_spin = false;
    }

    if (claw_reversing){
      claw.spin(reverse, 12, volt);
    }else if (r1){
      claw.spin(fwd, 12, volt);
    }else if (claw_idle_spin){
      claw.spin(fwd, 2, volt); // keep light pressure on the cup after intaking
    }else{
      claw.stop(hold);
    }



    

    

    // Active toggle (independent of the claw/lift logic):
    //   R1 held -> spins continuously, stops (hold) as soon as R1 is released
    //   B press -> one 360 degree turn, then holds
    bool b_pressed = button_b && !b_last_state;
    b_last_state = button_b;

    if (r1) {
      toggle_b_moving = false; // holding R1 overrides/cancels a B turn
      active_toggle.spin(fwd, 12, volt);
    } else if (b_pressed && !toggle_b_moving) {
      toggle_b_moving = true;
      active_toggle.setStopping(hold);
      active_toggle.spinFor(forward, 360, degrees, 100, velocityUnits::pct, false); // false = don't block the drive loop
    } else if (toggle_b_moving) {
      // let the 360 finish; the motor holds on its own when done
      if (active_toggle.isDone()) {
        toggle_b_moving = false;
      }
    } else {
      active_toggle.stop(hold);
    }

    wait(20, msec);

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

  resetChassis();

  vertical_tracker.resetPosition();
  lift.resetPosition();
  //chain_bar_1.resetPosition();

  // odom tracking (picks the setup from using_horizontal_tracker / using_vertical_tracker)
  startOdometry();

  // Optional: MCL uses the distance sensors to keep odometry from drifting while driving.
  // Needs field-centered coordinates, so start each auton with setPose() at the real field position.
  // startMcl();

  // Autonomous routines for the selector, in the order they appear on the Brain screen.
  // The first one is picked by default (until a choice is saved on the SD card).
  addAuton("near_side_auton", near_side_auton);
  addAuton("SKILLS", SKILLS);
  addAuton("farside", farside);
  addAuton("farside_3_cup_auton", farside_3_cup_auton);
  //addAuton("SKILLS", SKILLS);
  addAuton("Example", exampleAuton);
  addAuton("Move 2 in", movetwoinch); //pid  
  startAutonSelector();
}





