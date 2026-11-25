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

void RedRight() {
 //double matchloader
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
  driveTo(8.4, 800, false, 11);
  turnToAngle(-90, 800, false, 12);
  driveTo(23, 1000, true, 8);//preload
  //resetPositionBack();
  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  
  lift.setVelocity(50, velocityUnits::pct);
  lift.spinToPosition(45, degrees, true);
  claw.spin(reverse, 12, volt);
  lift.spinToPosition(100, degrees, true);
  wait(400, msec);
  //wait(1000, msec);
  //chain_bar_pnuematics.set(true); //drop preload in alliance goal
  //resetPositionLeft();

  driveTo(-8.7, 800, false, 9);
  claw.spin(fwd,12,volt);
  turnToAngle(-120, 400, true, 12);
  liftTo(0, false);
  driveTo(10, 700, true,12);
  driveChassis(4.6, 12);
  wait(450, msec);
  //driveChassis(0,0);
  //liftTo(50, true, 300);
  driveTo(3, 700, true, 8);
  driveChassis(4.6,12);
  wait(150, msec);
  driveChassis(0,0);
  driveChassis(4.6, 12);
  wait(100, msec);
  driveChassis(-2,-11);
  liftTo(50, true, 300);
  //wait(350, msec);
  turnToAngle(-90, 600, true, 12);
  liftTo(350, false);
  driveTo(-23, 1000, true, 12);
  resetPositionLeft();
  moveToPoint(-24, -48, 1, 1000, true,12);
  liftTo(50, true);
  claw.spin(reverse, 12, volt);

  driveTo(-10, 800, true, 12);
  turnToAngle(20, 600, true, 12);
  driveTo(26, 1000, true, 12);
  turnToAngle(0,700, true, 12);
  claw.spin(reverse, 12, volt);

  driveTo(4, 500, false, 5);
  claw.spin(fwd,12,volt);
  driveTo(3, 200, true, 8);
  liftTo(450, false, 800);
  driveTo(10, 800, true, 12);
  
  






  //driveTo(-10, 800, true, 12); //back out of alliance goal
  //boomerang(-20,-80,-1,120,0.3,1200,true,12);

  

  
  // chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  // boomerang(-41, 16, 1, 30, 0.3, 1100, true, 7);
  
  // turnToAngle(180,100,true,12);
  // chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  

  // driveTo(558, 500, false, 12);
  // turnToAngle(180, 1000, false, 12);
  // driveTo(15, 1000, false, 9);
  // driveTo(-5, 500, false, 10);
  // driveTo(5, 800, false, 8);
  // driveTo(-5, 300, false, 12);
  // resetPositionFront();
  // boomerang(-24, -25, 1, 45, 0.3, 1500, false, 10);


 
  //turnToAngle(50, 200, true, 5);
  //driveTo(10, 540, true, 5);
  //intake_pnuematics.set(false);
  //wait(400, msec);
  //driveTo(10, 400, true, 12);
  //turnToAngle(0, 1000, true, 12);
  //distanceReset('R', 'F', 334, 495);
  //moveToPoint(-2, -20, -1, 600, true, 12);


  
  //moveToPoint(-5, 25, 1, 750, true, 12);
  
}

//(-0.2,-63.7) is starting point with the toggle on the back
void BlueLeft(){
  setPose(-0.2, -63.7, 0);
  lift.setVelocity(100, velocityUnits::pct);
  lift.spinToPosition(200, degrees, true);
  wait(100, msec);
  lift.spinToPosition(0, degrees, true);
  // startMcl();
  resetPositionBack();
  //resetPositionLeft();
  //moveToPoint(0, -6, 1, 400, true, 12);
  driveTo(-6, 400, true, 12);//toggle #1
  driveTo(6, 300, true, 12);
  driveTo(-9, 450, true, 12);//toggle #2
  driveTo(5, 400, true, 12);
  resetPositionBack();
  lift.spinToPosition(250, degrees, false);


  //chain_bar_1.spinToPosition(52, degrees, 80, velocityUnits::pct, false);
  boomerang(22.2, -48.5, 1, 90, 0.4, 1500, true, 6);//drive to alliance goal
  resetPositionRight();
  lift.spinToPosition(0, degrees, true);
  claw.spin(reverse, 12, volt);
  //chain_bar_pnuematics.set(true);//drop preload in alliance goal
  wait(100, msec);
  driveTo(-10, 700, true, 12);//back out of allaince goal
  claw.spin(reverse, 0, volt);
  resetPositionRight();
  boomerang(19, -56.5, 1, 90, 0.4, 1300, true, 10);
  turnToAngle(90, 200, true, 12);
  resetPositionRight();
  driveTo(13, 800, true, 12);
  //chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  driveChassis(3, 12);
  wait(350, msec);
  driveChassis(0,0);
  turnToAngle(38.5, 500, true, 12);
  claw.spin(reverse, 12, volt);
  driveTo(5, 700, false, 2);//drive into it
  claw.spin(fwd, 12, volt);
  //chain_bar_pnuematics.set(false);//pick up pin
  driveTo(3.2, 300, true, 2);//drive into it
  wait(200, msec);
  //cascade.spinToPosition(1400, degrees, 90, velocityUnits::pct, true); //lift
  //chain_bar_1.spinToPosition(300, degrees, 50, velocityUnits::pct, false);
  lift.spinToPosition(450, degrees, false);
  boomerang(30, -45, 1, -90, 0.3, 2500, true, 6);
  wait(300, msec);
  //cascade.spinToPosition(600, degrees, 70, velocityUnits::pct, true); //lift
  //chain_bar_1.spinToPosition(150, degrees, 50, velocityUnits::pct, true);
  //chain_bar_pnuematics.set(true);
  lift.spinToPosition(0, degrees, true);
  wait(300, msec);
  claw.spin(reverse, 12, volt);
  wait(100, msec);
  driveTo(-17, 1800, false, 12);
  turnToAngle(180, 1000, true, 12);







}

//unfinished routmoveToPoint(emoveToPoint(
//(-0.2,-63.7) is starting point
void close_side_auto(){
  lift.setVelocity(100, velocityUnits::pct);
  lift.spinToPosition(30, degrees, false);
  setPose(-0.2, -62.7, 0);
  //lift.spinToPosition(220, degrees, true);
  //claw.spin(fwd, 6, volt);
  //wait(100, msec);
  //lift.spinToPosition(0, degrees, true);
  // startMcl();
  resetPositionBack();
  //resetPositionRight();
  //resetPositionLeft();
  //moveToPoint(0, -6, 1, 400, true, 12);
  active_toggle.spin(fwd,12,volt);
  wait(460, msec);
  lift.spinToPosition(210, degrees, false);
  //resetPositionBack();

  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  //boomerang(22, -49, 1, 90, 0.4, 1400, true, 8);//drive to alliance goal
  driveTo(7.6, 800, false, 9);
  turnToAngle(90, 700, false, 12);
  driveTo(24, 1000, true, 10);
  claw.spin(fwd, 0, volt);
  resetPositionRight();
  lift.setVelocity(50, velocityUnits::pct);
  lift.spinToPosition(20, degrees, true);
  wait(200,msec);
  claw.spin(reverse, 12, volt);
  wait(100, msec);
  lift.spinToPosition(90, degrees, false);
  driveTo(-12, 800, true, 12);//back out of allaince goal
  lift.spinToPosition(0, degrees, false);
  //resetPositionRight();
  lift.setVelocity(100, velocityUnits::pct);
  //chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  //turnToAngle(45, 550, false, 12);
  claw.spin(fwd, 12, volt);
  turnToAngle(40, 860, true, 9);
  boomerang(14.94, -33, 1, 30, 0.2, 1400, true, 7);//drive to the first pin stack and align
  //driveTo(6, 750, false, 5);//drive into it
  //wait(100, msec);
  //chain_bar_pnuematics.set(false);//got the first pin stack on allaiance line
  wait(150, msec); 
  claw.spin(reverse,12,volt);
  driveTo(8, 600, false, 3);//drive into it
  claw.spin(fwd,12,volt);
  driveTo(3, 500, true, 3);
  lift.setVelocity(70, velocityUnits::pct);
  lift.spinToPosition(30, degrees, true);
  lift.spinToPosition(460, degrees, false);
  //cascade.spinToPosition(1420, degrees, 90, velocityUnits::pct, false);
  turnToAngle(135, 1100, false, 9);
  boomerang(19.5, -48.5, 1, 186, 0.4, 1000, true, 11);//drive back to allaicne goal
  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  wait(100, msec);
  lift.setVelocity(60, velocityUnits::pct);
  claw.spin(fwd, 0, volt);
  lift.spinToPosition(70, degrees, true);
  claw.spin(reverse, 12, volt);
  wait(100, msec);
  //cascade.spinToPosition(550, degrees, 90, velocityUnits::pct, true); //lower b4 score
  //chain_bar_pnuematics.set(true);//score the pin stack on allaince goal
  //cascade.spinToPosition(1220, degrees, 90, velocityUnits::pct, false);

  driveTo(-8, 550, false, 10);
  lift.spinToPosition(0, degrees, false);
  //cascade.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  //chain_bar_1.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  turnToAngle(180, 400, true, 12);
  resetPositionLeft();
  //turnToAngle(155, 800, true, 12);
  //claw.spin(fwd, 12, volt);
  boomerang(32, -36, 1, 134, 0.3, 1500, true, 7);
  wait(300, msec);
  claw.spin(reverse, 12, volt);
  //driveTo(4.2, 700, false, 5);//drive into it
  //chain_bar_pnuematics.set(false);//pick up pin
  driveTo(13, 950, false, 5.5);//drive into it
  lift.setVelocity(60, velocityUnits::pct);
  claw.spin(fwd,12,volt);
  driveTo(5, 400, true, 5);
  lift.spinToPosition(50, degrees, true);
  lift.spinToPosition(960, degrees, false);
  //cascade.spinToPosition(800, degrees, 90, velocityUnits::pct, false);
  wait(120, msec);
  turnToAngle(-90, 800, true, 12);
  boomerang(20, -44.5, 1, -90, 0.3, 1000, true, 11);
  lift.spinToPosition(400, degrees, true);
  claw.spin(reverse,12,volt);
  lift.spinToPosition(700, degrees, true);
  wait(200, msec);
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
  driveTo(-10, 800, false, 9);
  turnToAngle(0, 800, true, 12);
  resetPositionBack();
  claw.spin(fwd, 12, volt);*/
 /* boomerang(-0.5, -26, 1, 0, 0.2, 900, false, 10);
  driveTo(7, 500, true, 5);
  wait(100, msec);
  lift.spinToPosition(500, degrees, false);
  boomerang(0,-5,1,0,0.2,900,true,11);
  lift.spinToPosition(0, degrees, true);
  claw.spin(fwd, 0, volt);
  wait(500, msec);

*/


  
}
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
  liftTo(200, false);
  //chkResetBack();

  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  //boomerang(22, -49, 1, 90, 0.4, 1400, true, 8);//drive to alliance goal
  chkDriveTo(7.9, 800, false, 9);
  turnToAngle(90, 700, false, 9);
  chkDriveTo(23.5, 1000, true, 12);//preload
  claw.spin(fwd, 0, volt);
  chkResetRight();
  lift.setVelocity(100, velocityUnits::pct);
  liftTo(20, true);
  //wait(200,msec);
  active_toggle.spin(fwd,0,volt);
  claw.spin(reverse, 12, volt);//drop preload
  wait(100, msec);
  liftTo(90, false);
  chkDriveTo(-8, 400, true, 12);//back out of allaince goal
  liftTo(0, false);
  //chkResetRight();
  lift.setVelocity(100, velocityUnits::pct);
  //chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  //turnToAngle(45, 550, false, 12);
  claw.spin(fwd, 12, volt);
  turnToAngle(34, 860, false, 9);
  chkDriveTo(17, 1000, true, 10);//allign to the first pin stack
  //boomerang(14.94, -33, 1, 30, 0.2, 1400, true, 7);//drive to the first pin stack and align
  //chkDriveTo(6, 750, false, 5);//drive into it
  //wait(100, msec);
  //chain_bar_pnuematics.set(false);//got the first pin stack on allaiance line
  //wait(150, msec); 
  claw.spin(reverse,12,volt);
  chkDriveTo(5, 600, false, 6);//drive into it
  claw.spin(fwd,12,volt);
  chkDriveTo(4, 500, true, 5);//intake
  lift.setVelocity(100, velocityUnits::pct);
  //liftTo(30, true);
  liftTo(340, false);
  //cascade.spinToPosition(1420, degrees, 90, velocityUnits::pct, false);
  turnToAngle(135, 1100, false, 9);
  boomerang(22, -48.5, 1, 186, 0.4, 1000, true, 11);//drive back to allaicne goal
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
  boomerang(37, -36, 1, 134, 0.3, 1350, true, 10);//prepare for second pin stack
  //wait(300, msec);
  claw.spin(reverse, 12, volt);
  //chkDriveTo(4.2, 700, false, 5);//drive into it
  //chain_bar_pnuematics.set(false);//pick up second pin stack
  chkDriveTo(10, 850, false, 5);//drive into it
  //chkDriveTo(3, 300, false, 7);
  lift.setVelocity(100, velocityUnits::pct);
  claw.spin(fwd,12,volt);//grab it
  chkDriveTo(8, 420, true, 5);
  liftTo(150, false);
  

  moveToPoint(24, -22.4, 1, 1050, false, 12);//get out of area
  liftTo(400, false);
  boomerang(-22, -50, 1, -90, 0.3, 1400, false, 12);//allign to neutral goal
  driveTo(5, 300, true, 9);
  liftTo(250, true, 300);
  claw.spin(reverse, 12, volt);//score

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





