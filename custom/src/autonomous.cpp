#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>


#include "../include/autonomous.h"
#include "../include/robot-config.h"
#include "motor-control.h"
#include "../include/field-resets.h"
#include "../include/lift.h"
#include "../include/auton-debug.h"

// Adding a new auton:
// 1. Write the function in this file.
// 2. Declare it in custom/include/autonomous.h (e.g. void MyNewAuton();).
// 3. Add it to the selector in runPreAutonomous() in custom/src/user.cpp,
//    before startAutonSelector(): addAuton("My New Auton", MyNewAuton);
// Notes: the selector holds up to 12 autons, names fit in about 14 characters,
// and field-position autons should start with setPose() at the real start spot.
// Test in the simulator with: sh sim/run-sim.sh "My New Auton"



// distanceReset helpers are in custom/src/field-resets.cpp


// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }





//fix this function so that it can do the roller first and then move 2 inches
//PID
void movetwoinch(){
  driveTo(24, 1500, true, 12);
  //turnToAngle(90, 1300, true, 12);
}

//not gonna use (testing purposes) 
//testing purposes
void exampleAuton() {
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  Brain.Screen.print("Left Side: %.1f deg | Right Side: %.1f deg", 
                     left_chassis.position(deg), 
                     right_chassis.position(deg)); 
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  Brain.Screen.print("Left Side: %.1f deg | Right Side: %.1f deg", 
                     left_chassis.position(deg), 
                     right_chassis.position(deg)); 
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  Brain.Screen.print("Left Side: %.1f deg | Right Side: %.1f deg", 
                     left_chassis.position(deg), 
                     right_chassis.position(deg)); 
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}

//preload and middle cup
void farside(){
  setPose(-0.2, -63.7, 0);
  // startMcl();
  resetPositionBack();
  liftTo(30, true, 300);
  lift.setVelocity(100, velocityUnits::pct);
  //lift.spinToPosition(200, degrees, true);
  active_toggle.spin(fwd,12,volt);
  wait(260, msec);
  liftTo(260, false);
  wait(200, msec);
  driveTo(8.6, 800, false, 11);
  turnToAngle(-90, 800, false, 12);
  driveTo(23, 1000, true, 8);//preload
  wait(1000, msec);
  lift.setVelocity(50, velocityUnits::pct);
  lift.spinToPosition(45, degrees, true);
  claw.spin(reverse, 12, volt);
  lift.spinToPosition(100, degrees, true);
  wait(400, msec);
  driveTo(-8, 800, false, 9);
  claw.spin(fwd,12,volt);
  turnToAngle(0, 900, true, 12);
  liftTo(0, false);
  driveTo(13.5, 700, true,12);
  claw.spin(reverse,12,volt);
  driveTo(5, 300, false, 5);
  claw.spin(fwd,12,volt);
  driveTo(5, 400, true, 5);
  liftTo(450, true);
  driveTo(15, 1000, true, 12);
  liftTo(100, true);
  claw.spin(reverse,12,volt);

  //resetPositionBack();
}

//scores preload and 3 more cups while almost getting local awp
// Mirror of near_side_auton across the field's center line (x = 0):
//   x -> -x, y unchanged, heading -> -heading (36 -> -36, 90 -> -90, 180 -> -180, 0 stays 0),
//   and the LEFT/RIGHT distance resets swap (the wall the right sensor saw is now on the left).
//   driveTo distances, moveToPoint directions, lift and claw are unchanged.
void farside_3_cup_auton(){
  startAutonDebug("FAR 3 CUP");
  lift.setVelocity(100, velocityUnits::pct);
  liftTo(30, false);
  claw.spin(fwd,12,volt);
  setPose(0.2, -62.7, 0);
  chkResetBack();
  active_toggle.spin(fwd,12,volt);
  wait(400, msec);
  liftTo(206, false);

  chkDriveTo(7.9, 800, false, 9);
  boomerang(-23.5, -47.6, 1, -90, 0.4, 1200, true, 10);//preload

  claw.spin(fwd, 0, volt);
  chkResetLeft();                              // near side: chkResetRight
  lift.setVelocity(100, velocityUnits::pct);
  liftTo(20, true);
  active_toggle.spin(fwd,0,volt);
  claw.spin(reverse, 12, volt);//drop preload
  wait(100, msec);
  liftTo(90, false);
  chkDriveTo(-7.67, 400, true, 12);//back out of allaince goal
  liftTo(0, false);
  lift.setVelocity(100, velocityUnits::pct);
  claw.spin(fwd, 12, volt);
  turnToAngle(-36, 860, false, 9);
  chkDriveTo(17, 1000, true, 10);//allign to the first pin stack
  claw.spin(reverse,12,volt);
  chkDriveTo(6, 600, false, 4);//drive into it
  claw.spin(fwd,12,volt);                     //grab 1st pin and cup
  liftTo(340, false);
  chkDriveTo(5.5, 500, true, 5);//intake
  lift.setVelocity(100, velocityUnits::pct);
  turnToAngle(-135, 1100, false, 9);
  boomerang(-22.5, -46, 1, -186, 0.4, 900, true, 11);//drive back to allaicne goal
  lift.setVelocity(100, velocityUnits::pct);
  claw.spin(fwd, 0, volt);
  liftTo(140, true);
  claw.spin(reverse, 12, volt);//score the first pin stack

  chkDriveTo(-8, 550, false, 10);
  liftTo(0, false);
  turnToAngle(-180, 400, true, 12);
  chkResetRight();                             // near side: chkResetLeft
  boomerang(-37.2, -36, 1, -134, 0.2, 1350, true, 10);//prepare for second pin stack
  claw.spin(reverse, 12, volt);
  chkDriveTo(13, 960, false, 6);//drive into it
  lift.setVelocity(100, velocityUnits::pct);
  claw.spin(fwd,12,volt);//grab it
  chkDriveTo(7, 420, true, 4);
  liftTo(150, false);

  moveToPoint(-24, -22.4, 1, 1050, false, 12);//get out of area
  liftTo(400, false);
  boomerang(22, -51, 1, 90, 0.3, 1450, false, 12);//allign to neutral goal
  driveTo(5, 300, true, 7);
  chkResetRight();                             // near side: chkResetLeft
  liftTo(250, true, 300);
  claw.spin(reverse, 12, volt);//score

  moveToPoint(0,-48,-1,700, false, 12);
  liftTo(0, false);
  turnToAngle(0, 650, true, 12);
  moveToPoint(-1.2,-28,1,500,false,12);
  claw.spin(reverse,12,volt);
  driveTo(4.5, 500, false, 5);
  claw.spin(fwd, 12, volt);
  liftTo(500, false);
  driveTo(3.6, 350, true, 3.5);
  moveToPoint(-2.2,-4.6,1,500,false,12);
  wait(150, msec);
  lift.setVelocity(100, velocityUnits::pct);
  liftTo(350, false, 500);
}

//(-0.2,-63.7) is starting point with the toggle on the back


//split line cups on alliance goal and neutral goal as well as score middle cup. 
void near_side_auton(){
  startAutonDebug("NEAR SIDE");
  lift.setVelocity(100, velocityUnits::pct);
  liftTo(30, false);
  claw.spin(fwd,12,volt);
  setPose(-0.2, -62.7, 0);
  //lift.spinToPosition(220, degrees, true);
  //claw.spin(fwd, 6, volt);
  //wait(100, msec);
  //lift.spinToPosition(0, degrees, true);
  // startMcl();
  chkResetBack();
  //chkResetRight();
  //chkResetLeft();
  //moveToPoint(0, -6, 1, 400, true, 12);
  active_toggle.spin(fwd,12,volt);
  wait(400, msec);
  liftTo(206, false);
  //chkResetBack();

  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  //boomerang(22, -49, 1, 90, 0.4, 1400, true, 8);//drive to alliance goal
  chkDriveTo(7.9, 800, false, 9);
  //turnToAngle(90, 700, false, 9);
  //chkDriveTo(23.5, 1000, true, 12);//preload
  boomerang(23.5, -47.6, 1, 90, 0.4, 1200, true, 10);//preload

  claw.spin(fwd, 0, volt);
  chkResetRight();
  lift.setVelocity(100, velocityUnits::pct);
  liftTo(20, true);
  //wait(200,msec);
  active_toggle.spin(fwd,0,volt);
  claw.spin(reverse, 12, volt);//drop preload
  wait(100, msec);
  liftTo(90, false);
  chkDriveTo(-7.67, 400, true, 12);//back out of allaince goal
  liftTo(0, false);
  //chkResetRight();
  lift.setVelocity(100, velocityUnits::pct);
  //chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  //turnToAngle(45, 550, false, 12);
  claw.spin(fwd, 12, volt);
  turnToAngle(36, 860, false, 9);
  chkDriveTo(17, 1000, true, 10);//allign to the first pin stack
  //boomerang(14.94, -33, 1, 30, 0.2, 1400, true, 7);//drive to the first pin stack and align
  //chkDriveTo(6, 750, false, 5);//drive into it
  //wait(100, msec);
  //chain_bar_pnuematics.set(false);//got the first pin stack on allaiance line
  //wait(150, msec); 
  claw.spin(reverse,12,volt);
  chkDriveTo(6, 600, false, 4);//drive into it
  claw.spin(fwd,12,volt);                     //grab 1st pin and cup
  liftTo(340, false);
  chkDriveTo(5.5, 500, true, 5);//intake
  lift.setVelocity(100, velocityUnits::pct);
  //liftTo(30, true);
  //cascade.spinToPosition(1420, degrees, 90, velocityUnits::pct, false);
  turnToAngle(135, 1100, false, 9);
  boomerang(22.5, -46, 1, 186, 0.4, 900, true, 11);//drive back to allaicne goal
  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  //wait(100, msec);
  lift.setVelocity(100, velocityUnits::pct);
  claw.spin(fwd, 0, volt);
  liftTo(140, true);
  claw.spin(reverse, 12, volt);//score the first pin stack
  //wait(100, msec);
  //cascade.spinToPosition(550, degrees, 90, velocityUnits::pct, true); //lower b4 score
  //chain_bar_pnuematics.set(true);//score the pin stack on allaince goal
  //cascade.spinToPosition(1220, degrees, 90, velocityUnits::pct, false);

  chkDriveTo(-8, 550, false, 10);
  liftTo(0, false);
  //cascade.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  //chain_bar_1.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  turnToAngle(180, 400, true, 12);
  chkResetLeft();
  //turnToAngle(155, 800, true, 12);
  //claw.spin(fwd, 12, volt);
  boomerang(37.2, -36, 1, 134, 0.2, 1350, true, 10);//prepare for second pin stack
  //wait(300, msec);
  claw.spin(reverse, 12, volt);
  //chkDriveTo(4.2, 700, false, 5);//drive into it
  //chain_bar_pnuematics.set(false);//pick up second pin stack
  chkDriveTo(13, 960, false, 6);//drive into it
  //chkDriveTo(3, 300, false, 7);
  lift.setVelocity(100, velocityUnits::pct);
  claw.spin(fwd,12,volt);//grab it
  chkDriveTo(7, 420, true, 4);
  liftTo(150, false);
  

  moveToPoint(24, -22.4, 1, 1050, false, 12);//get out of area
  liftTo(400, false);
  boomerang(-22, -51, 1, -90, 0.3, 1450, false, 12);//allign to neutral goal
  driveTo(5, 300, true, 7);
  chkResetLeft();
  liftTo(250, true, 300);
  claw.spin(reverse, 12, volt);//score

  moveToPoint(0,-48,-1,700, false, 12);
  liftTo(0, false);
  turnToAngle(0, 650, true, 12);
  moveToPoint(1.2,-28,1,500,false,12);
  claw.spin(reverse,12,volt);
  driveTo(4.5, 500, false, 5);
  claw.spin(fwd, 12, volt);
  liftTo(500, false);
  //liftTo(60, false);
  driveTo(3.6, 350, true, 3.5);
  moveToPoint(2.2,-4.6,1,500,false,12); 
  wait(150, msec);
  lift.setVelocity(100, velocityUnits::pct);
  liftTo(350, false, 500);
  //claw.spin(reverse,12,volt);


    /*
  driveTo(-10, 800, false,12);
  liftTo(0, false);
  turnToAngle(0, 700, true, 12);
  driveTo(10, 1000, false, 12);
  driveTo(4, 400, false, 6);
  claw.spin(fwd,12,volt);
  //liftTo(30, true, 200);
  liftTo(500, false);
  driveTo(3, 200, true, 6);
  //liftTo(600, false);
  driveTo(20, 800, true, 12);
  liftTo(0, true);



  
  /*liftTo(700, false);
  //cascade.spinToPosition(800, degrees, 90, velocityUnits::pct, false);
  wait(120, msec);
  turnToAngle(-90, 800, true, 12);
  boomerang(26, -45.5, 1, -90, 0.3, 1458, true, 8);//score second pin stack point
  wait(200, msec);
  claw.spin(fwd,2,volt);
  lift.setVelocity(50, velocityUnits::pct);
  liftTo(400, true, 950);
  claw.spin(reverse,12,volt);//score second pin stack point
  //lift.spinToPosition(700, degrees, true);
  wait(500, msec);
  liftTo(0, false);
  chkDriveTo(-35.5, 1200, true, 12); //backout of the alliance goal after scoring the second line cup
  checkpoint("out2");
  Brain.Screen.print("HI ANISH");
  // lift motor diagnostics: connected?, temp (C), current (A), own encoder (deg)  
  Brain.Screen.setCursor(10, 1);
  claw.spin(fwd,12,volt);*/
}

//Elims auton route as well and skill route for 88-108
void SKILLS(){
  startAutonDebug("SKILLS");
  lift.setVelocity(100, velocityUnits::pct);
  liftTo(30, false);
  setPose(-0.2, -62.7, 0);
  //lift.spinToPosition(220, degrees, true);
  //claw.spin(fwd, 6, volt);
  //wait(100, msec);
  //lift.spinToPosition(0, degrees, true);
  // startMcl();
  chkResetBack();
  //chkResetRight();
  //chkResetLeft();
  //moveToPoint(0, -6, 1, 400, true, 12);
  active_toggle.spin(fwd,12,volt);
  wait(460, msec);
  liftTo(210, false);
  //chkResetBack();

  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  //boomerang(22, -49, 1, 90, 0.4, 1400, true, 8);//drive to alliance goal
  chkDriveTo(7.7, 800, false, 9);
  turnToAngle(90, 700, false, 12);
  chkDriveTo(23.5, 1000, true, 8);//preload
  claw.spin(fwd, 0, volt);
  chkResetRight();
  lift.setVelocity(50, velocityUnits::pct);
  liftTo(20, true);
  wait(200,msec);
  active_toggle.spin(fwd,0,volt);
  claw.spin(reverse, 12, volt);//drop preload
  wait(100, msec);
  liftTo(90, false);
  chkDriveTo(-8, 450, true, 12);//back out of allaince goal
  liftTo(0, false);
  //chkResetRight();
  lift.setVelocity(100, velocityUnits::pct);
  //chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  //turnToAngle(45, 550, false, 12);
  claw.spin(fwd, 12, volt);
  turnToAngle(34, 860, true, 9);
  chkDriveTo(17, 1000, true, 12);//allign to the first pin stack
  //boomerang(14.94, -33, 1, 30, 0.2, 1400, true, 7);//drive to the first pin stack and align
  //chkDriveTo(6, 750, false, 5);//drive into it
  //wait(100, msec);
  //chain_bar_pnuematics.set(false);//got the first pin stack on allaiance line
  wait(150, msec); 
  claw.spin(reverse,12,volt);
  chkDriveTo(4, 600, false, 3);//drive into it
  claw.spin(fwd,12,volt);
  chkDriveTo(3, 500, true, 3);//intake
  lift.setVelocity(70, velocityUnits::pct);
  liftTo(30, true);
  liftTo(400, false);
  //cascade.spinToPosition(1420, degrees, 90, velocityUnits::pct, false);
  turnToAngle(135, 1100, false, 9);
  boomerang(22, -48.5, 1, 186, 0.4, 1000, true, 11);//drive back to allaicne goal
  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  wait(100, msec);
  lift.setVelocity(60, velocityUnits::pct);
  claw.spin(fwd, 0, volt);
  liftTo(70, true);
  claw.spin(reverse, 12, volt);//score the first pin stack
  wait(100, msec);
  //cascade.spinToPosition(550, degrees, 90, velocityUnits::pct, true); //lower b4 score
  //chain_bar_pnuematics.set(true);//score the pin stack on allaince goal
  //cascade.spinToPosition(1220, degrees, 90, velocityUnits::pct, false);

  chkDriveTo(-8, 550, false, 10);
  liftTo(0, false);
  //cascade.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  //chain_bar_1.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  turnToAngle(180, 400, true, 12);
  chkResetLeft();
  //turnToAngle(155, 800, true, 12);
  //claw.spin(fwd, 12, volt);
  boomerang(34.5, -36, 1, 134, 0.3, 1500, true, 7);//prepare for second pin stack
  wait(300, msec);
  claw.spin(reverse, 12, volt);
  //chkDriveTo(4.2, 700, false, 5);//drive into it
  //chain_bar_pnuematics.set(false);//pick up second pin stack
  chkDriveTo(13, 950, false, 5.5);//drive into it
  lift.setVelocity(75, velocityUnits::pct);
  claw.spin(fwd,12,volt);
  chkDriveTo(5, 400, true, 5);
  liftTo(50, true);
  liftTo(700, false);
  //cascade.spinToPosition(800, degrees, 90, velocityUnits::pct, false);
  wait(120, msec);
  turnToAngle(-90, 800, true, 12);
  boomerang(26, -45.5, 1, -90, 0.3, 1458, true, 8);//score second pin stack point
  wait(200, msec);
  claw.spin(fwd,2,volt);
  lift.setVelocity(50, velocityUnits::pct);
  liftTo(400, true, 950);
  claw.spin(reverse,12,volt);//score second pin stack point
  //lift.spinToPosition(700, degrees, true);
  wait(500, msec);
  liftTo(0, false);
  chkDriveTo(-35.5, 1200, true, 12); //backout of the alliance goal after scoring the second line cup
  checkpoint("out2");
  Brain.Screen.print("HI ANISH");
  // lift motor diagnostics: connected?, temp (C), current (A), own encoder (deg)  
  Brain.Screen.setCursor(10, 1);
  claw.spin(fwd,12,volt);
  turnToAngle(180, 600, true, 12);
  chkResetLeft();
  chkDriveTo(14.8, 950, true, 9);//matchload #1
  //wait(200, msec);
  liftTo(50, true);
  liftTo(775, false);
  chkDriveTo(-6, 600, false, 12);
  boomerang(27, -44.5, 1, -90, 0.3, 1758, true, 6);//scoring the first matchloader point
  wait(450, msec);
  lift.setVelocity(50, velocityUnits::pct);
  liftTo(530, true, 1300);
  wait(200, msec);
  claw.spin(reverse, 12, volt);
  //wait(300, msec);
  liftTo(0, false);
  chkDriveTo(-35.5, 1200, true, 12);
  checkpoint("out3");
  claw.spin(fwd,12,volt);
  turnToAngle(180, 600, true, 12);
  //chkResetLeft();
  chkDriveTo(14.3, 950, true, 8); //preload get
  checkpoint("preld");
  //wait(200, msec);
  liftTo(50, true, 600);
  chkDriveTo(-8, 600, false, 12);
  liftTo(320, false);
  moveToPoint(49.6, -39, -1, 700, true, 12);//move 
  turnToAngle(0, 1000, true, 12);//fix angle
  chkResetRight();
  driveTo(14, 700, true, 12);
  //boomerang(49, -22, 1, 0, 0.3, 800, true, 9);//score
  checkpoint("score4");
  liftTo(50, true);
  claw.spin(reverse, 12, volt);
  wait(350, msec);
  liftTo(100, false);
  chkDriveTo(-8, 800, true, 12);
  turnToAngle(180, 1000, true, 12);
  liftTo(0, false);
  checkpoint("preDR");
  chkDistanceReset('L', 'F', 377, 690);
  checkpoint("postDR");
  claw.spin(fwd,12,volt);
  moveToPoint(20, -8, 1, 900, true, 12);//go to matchloader
  checkpoint("mtp1");
  turnToAngle(185, 900, true, 12);
  chkDriveTo(8, 700, true, 12);//pick it up
  checkpoint("pickup");
  //chkResetLeft();
  checkpoint("rstL");
  //claw.spin(fwd,0,volt);
  liftTo(50, true, 500);
  checkpoint("lift50");
  //wait(200, msec);
  driveChassis(-12,-12);
  liftTo(410, false);
  wait(230, msec);
  driveChassis(0,0);
  //chkDriveTo(-80, 5600, true, 12);
  checkpoint("back8");
  moveToPoint(11, 6, 1, 900, true, 10);
  turnToAngle(4, 700, true, 12);
  driveTo(8, 600, true, 12);
  //boomerang(2, 26, 1, 0, 0.3, 800, true, 12);
  liftTo(60, true, 670);
  claw.spin(reverse, 12, volt);
  chkDriveTo(-10, 800, true, 12);
  turnToAngle(180, 1000, true, 12);
  liftTo(0, false);
  checkpoint("preDR");
  chkDistanceReset('L', 'F', 377, 690);
  checkpoint("postDR");
  claw.spin(fwd,12,volt);
  moveToPoint(12.5, -10, 1, 900, true, 12);//go to matchloader
  checkpoint("mtp1");
  turnToAngle(178, 900, true, 12);
  chkDriveTo(9, 700, true, 12);//pick it up
  checkpoint("pickup");
  //chkResetLeft();
  checkpoint("rstL");
  //claw.spin(fwd,0,volt);
  liftTo(50, true, 500);
  checkpoint("lift50");
  //wait(200, msec);
  driveChassis(-12,-12);
  liftTo(650, false);
  wait(400, msec);
  driveChassis(0,0);
  //chkDriveTo(-80, 5600, true, 12);
  checkpoint("back8");
  moveToPoint(6, 10, 1, 700, true, 12);
  turnToAngle(0, 700, true, 12);
  boomerang(6, 26, 1, 0, 0.3, 800, true, 12);
  liftTo(260, true, 670);
  claw.spin(reverse, 12, volt);


  chkDriveTo(-7, 500, false, 12);
  claw.spin(fwd, 12, volt);
  turnToAngle(180, 1000, true, 12);
  liftTo(0, false);
  resetPositionLeft();
  turnToAngle(160, 300, false, 12);
  chkDriveTo(10, 600, false, 12);
  turnToAngle(180, 700, false, 12);
  chkDriveTo(10, 700, true, 12);
  liftTo(50, true, 400);
  wait(200, msec);
  chkDriveTo(-10, 600, true, 12);

  turnToAngle(0, 1000, true, 12);
  resetPositionRight();
  chkDistanceReset('R', 'B', 150, 410);
  liftTo(775, false);
  boomerang(-12, 24, 1, 0, 0.3, 800, true, 9);//score
  driveTo(8, 600, true, 12);
  liftTo(530, true, 1300);
  claw.spin(reverse, 12, volt);

  chkDriveTo(-10, 600, false, 12);
  claw.spin(fwd,12,volt);
  turnToAngle(180, 800,true,12);
  liftTo(0, false);
  resetPositionLeft();
  boomerang(0,0,1,180,0.3,800,true,10);
  driveTo(9, 600, true,12);
  liftTo(50, true);

  chkDriveTo(-10, 600, true, 12);

  /*turnToAngle(0, 1000, true, 12);
  resetPositionRight();
  chkDistanceReset('R', 'B', 150, 410);
  liftTo(975, false);//fix this later
  boomerang(-12, 24, 1, 0, 0.3, 800, true, 9);//score
  driveTo(8, 600, true, 12);
  liftTo(730, true, 1300);
  claw.spin(reverse, 12, volt);*/

  /*chkDriveTo(-10, 600, false, 12);
  claw.spin(fwd,12,volt);
  turnToAngle(180, 800,true,12);
  liftTo(0, false);
  resetPositionLeft();
  boomerang(0,0,1,180,0.3,800,true,10);
  chkDriveTo(9, 600, true,12);
  liftTo(50, true);*/

  //chkDriveTo(-10, 600, false, 12);
  turnToAngle(200, 400, false, 12);
  chkDriveTo(8, 600, false, 12);
  turnToAngle(180, 500, true,12);
  resetPositionFront();
  resetPositionLeft();
  active_toggle.spin(fwd,12,volt);
  chkDriveTo(50, 2000, true,12);
  driveChassis(12,3);
  wait(300, msec);
  liftTo(560, false);
  driveChassis(0,0);
  turnToAngle(-90, 500, true,12);
  boomerang(0,0, 1,-900.3, 1200,true,12);
  liftTo(300, true,660);
  claw.spin(reverse,12,volt);
  driveTo(-5, 500, true,12);
  liftTo(0,true);




  
  /*checkpoint("mtp2"); // compare x/y here to the target (46.5, 39)
  turnToAngle(180, 1000, true, 12);
  checkpoint("turn");
  liftTo(30, true);
  checkpoint("END");
  //left 377, front 690


  */


  

  /*moveToPoint(17, -13, -1, 900, false, 10);
  //moveToPoint(-18, -60, 1, 1000, false, 12);
  //turnToAngle(-90, 200, true, 12);
  //cascade.spinToPosition(1450, degrees, 90, velocityUnits::pct, false);
  //chain_bar_1.spinToPosition(60, degrees, 100, velocityUnits::pct, false);
  lift.spinToPosition(200, degrees, false);
  lift.setVelocity(45, velocityUnits::pct);
  boomerang(-28, -45.5, 1, 270, 0.4, 1650, true, 11);
  claw.spin(fwd, 0, volt);
  lift.spinToPosition(70, degrees, true);
  //cascade.spinToPosition(800, degrees, 100, velocityUnits::pct, true); //lower b4 score
  //chain_bar_pnuematics.set(true);//pick up pin
  chkDriveTo(-10, 800, false, 9);
  turnToAngle(0, 800, true, 12);
  chkResetBack();
  claw.spin(fwd, 12, volt);*/
 /* boomerang(-0.5, -26, 1, 0, 0.2, 900, false, 10);
  chkDriveTo(7, 500, true, 5);
  wait(100, msec);
  lift.spinToPosition(500, degrees, false);
  boomerang(0,-5,1,0,0.2,900,true,11);
  lift.spinToPosition(0, degrees, true);
  claw.spin(fwd, 0, volt);
  wait(500, msec);

  */

  
}





