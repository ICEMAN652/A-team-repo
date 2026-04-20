#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"




// Modify autonomous, driver, or pre-auton code below




void runAutonomous() {
  int auton_selected = 9;
  switch(auton_selected) {
    case 1:
      autonskillsActual();
      break;
    case 2:
      autonskills(); //newly made
      break;
    case 3:
      rightsidelow();
      break;
    case 4:
      leftandmid();
      break;
    case 5:
      leftside7();
      break;
    case 6:
      movetwoinch();
      break;
    case 7:
      rightsidepush();
      break;
    case 8:
      rightside4push();
      break;
    case 9:
      rightside2();
      break;
    case 10:
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
   // Set brake mode to coast at the start of driver control
  stopChassis(coast);
  heading_correction = false;

  while (true) {
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

       // 1. Get Joystick Inputs (PROS scale: -100 to 100)
       double forwardInput = (double)controller_1.Axis3.value();
       double turnInput = (double)controller_1.Axis1.value();//Norrel was here


       // 2. Curvature Drive Logic
       const double TURN_REDUCTION = 0.5;  
       const double TURN_BOOST = 0.5;      


       // Normalize to [-1.0, 1.0] for the math
       double f = forwardInput / 127.0;
       double t = turnInput / 127.0;


       // Apply scaling (Curvature Drive)
       double turnScale = 1.0 - (TURN_REDUCTION * fabs(t));
       f *= turnScale;


       double speedBoost = 1.0 + (TURN_BOOST * fabs(f));
       t *= speedBoost;


       // 3. Convert back to motor units (-127 to 127)
       int leftOutput = (int)((f + t) * 127);
       int rightOutput = (int)((f - t) * 127);


       // 4. Move the Chassis
       driveChassis(leftOutput,rightOutput);


      //intake logic 
      if(r1){
        intake1.spin(fwd,12,volt);
      } else if(r2){
        intake1.spin(fwd, 12,volt);
        intake2.spin(fwd, 12,volt);
      } else if(l2){
        intake1.spin(reverse,12,volt);
        intake2.spin(reverse,12,volt);
      } else {
        intake1.spin(fwd,0,volt);
        intake2.spin(fwd,0,volt);
      }
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





