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
void movetwoinch(){
  //driveTo(24, 1500, true, 12);
  turnToAngle(90, 1300, true, 12);
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
//(64.4,0.2) is starting point with toggle odom pod on the line, intake facing center goal
void BlueLeftAutonTwo() {
  driveTo(558, 500, true, 12);
  turnToAngle(180, 1000, true, 12);
  driveTo(15, 1000, true, 9);
  driveTo(-5, 500, true, 10);
  driveTo(5, 800, true, 8);
  driveTo(-5, 300, true, 12);
  // wait(500,msec);
  // moveToPoint(24,-30,1,1000,true,10);
  

}

//(-0.2,-63.7) is starting point with the toggle on the back

void RedRight() {
 //double matchloader
  setPose(-0.2, -63.7, 0);
  // startMcl();
  resetPositionBack();
  lift.setVelocity(100, velocityUnits::pct);
  lift.spinToPosition(200, degrees, true);
  wait(100, msec);
  lift.spinToPosition(0, degrees, true);
  driveTo(-6, 400, true, 12);//toggle #1
  driveTo(6, 300, true, 12);
  driveTo(-9, 450, true, 12);//toggle #2
  driveTo(5, 400, true, 12);
  lift.spinToPosition(240, degrees, false);
  wait(200, msec);

  //resetPositionBack();
  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  driveTo(12, 800, true, 12);
  turnToAngle(-90, 800,true,12);
  boomerang(-20.5, -47.5, 1, 90, 0.4, 1400, true, 7);//drive to alliance goal
  // resetPositionRight();
  lift.setVelocity(50, velocityUnits::pct);
  lift.spinToPosition(45, degrees, true);
  claw.spin(reverse, 4, volt);
  lift.spinToPosition(100, degrees, true);
  wait(1000, msec);
  //chain_bar_pnuematics.set(true); //drop preload in alliance goal
  //resetPositionLeft();

 moveToPoint(0, -58.4, -1, 1000, false, 9); //drive back to prepare
 lift.spinToPosition(0, degrees, false);
  turnToAngle(-90,500,true,12); //straighten out
  resetPositionLeft();
  claw.spin(fwd, 12, volt);
  //boomerang(-0, -63.5, -1, 90, -0.6, 1200, true, 10);//drive to alliance goal
  boomerang(-58.8, -61.7, 1, 90, 0.8, 1100, true, 6.5);//drive to pin
  turnToAngle(-46, 700, true, 12); //turn to righ to orientation for pin


  driveTo(10, 900, false, 2.6);//drive into pin
  driveTo(5, 500, true, 4);//drive into it
  wait(200, msec);
  lift.spinToPosition(450, degrees, false);
  turnToAngle(94, 700, true, 2.5); //turn to righ to orientation for pin
  //resetPositionRight();
  driveTo(23, 1200, true, 7);
  lift.spinToPosition(0, degrees, true);
  wait(100, msec);

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
  resetPositionRight();
  boomerang(19, -56.5, 1, 90, 0.4, 1300, true, 10);
  turnToAngle(90, 200, true, 12);
  resetPositionRight();
  claw.spin(fwd, 12, volt);
  driveTo(13, 800, true, 12);
  //chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  driveChassis(3, 12);
  wait(350, msec);
  driveChassis(0,0);
  turnToAngle(38.5, 500, true, 12);
  driveTo(5, 700, false, 2);//drive into it
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
  driveTo(-17, 1800, false, 12);
  turnToAngle(180, 1000, true, 12);







}

//unfinished routmoveToPoint(emoveToPoint(
//(-0.2,-63.7) is starting point
void SAWP(){
  lift.setVelocity(100, velocityUnits::pct);
  setPose(-0.2, -62.7, 0);
  lift.spinToPosition(200, degrees, true);
  wait(100, msec);
  lift.spinToPosition(0, degrees, true);
  // startMcl();
  resetPositionBack();
  //resetPositionRight();
  //resetPositionLeft();
  //moveToPoint(0, -6, 1, 400, true, 12);
  driveTo(-6, 400, true, 12);//toggle #1
  driveTo(6, 300, true, 12);
  driveTo(-9, 450, true, 12);//toggle #2
  driveTo(5, 400, true, 12);
  lift.spinToPosition(230, degrees, false);
  resetPositionBack();

  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  boomerang(19, -52, 1, 90, 0.4, 1300, true, 10);//drive to alliance goal
  resetPositionRight();
  lift.setVelocity(70, velocityUnits::pct);
  lift.spinToPosition(0, degrees, true);
  claw.spin(reverse, 5, volt);
  wait(100, msec);
  driveTo(-10, 700, true, 12);//back out of allaince goal
  resetPositionRight();
  lift.setVelocity(100, velocityUnits::pct);
  //chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  //turnToAngle(45, 550, false, 12);
  claw.spin(fwd, 12, volt);
  boomerang(18, -33, 1, 30, 0.3, 1200, true, 9);//drive to the first pin stack and align
  //driveTo(6, 750, false, 5);//drive into it
  //wait(100, msec);
  //chain_bar_pnuematics.set(false);//got the first pin stack on allaiance line
  wait(150, msec);
  driveTo(11, 750, true, 5);//drive into it
  lift.spinToPosition(400, degrees, false);
  //cascade.spinToPosition(1420, degrees, 90, velocityUnits::pct, false);
  turnToAngle(135, 1100, false, 10);
  boomerang(23, -46.5, 1, 180, 0.4, 1000, true, 11);//drive back to allaicne goal
  //chain_bar_1.spinToPosition(50, degrees, 80, velocityUnits::pct, false);
  wait(100, msec);
  lift.setVelocity(60, velocityUnits::pct);
  claw.spin(fwd, 0, volt);
  lift.spinToPosition(0, degrees, true);
  //cascade.spinToPosition(550, degrees, 90, velocityUnits::pct, true); //lower b4 score
  //chain_bar_pnuematics.set(true);//score the pin stack on allaince goal
  //cascade.spinToPosition(1220, degrees, 90, velocityUnits::pct, false);

  driveTo(-8, 550, false, 10);
  //cascade.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  //chain_bar_1.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  turnToAngle(180, 400, true, 12);
  resetPositionLeft();
  //turnToAngle(155, 800, true, 12);
  claw.spin(fwd, 12, volt);
  boomerang(37.5, -38, 1, 134, 0.3, 1500, true, 7);
  wait(500, msec);
  //driveTo(4.2, 700, false, 5);//drive into it
  //chain_bar_pnuematics.set(false);//pick up pin
  driveTo(11, 800, true, 5);//drive into it
  //cascade.spinToPosition(800, degrees, 90, velocityUnits::pct, false);
  wait(120, msec);
  lift.spinToPosition(200, degrees, false);
  moveToPoint(17, -13, -1, 900, false, 10);
  //moveToPoint(-18, -60, 1, 1000, false, 12);
  //turnToAngle(-90, 200, true, 12);
  //cascade.spinToPosition(1450, degrees, 90, velocityUnits::pct, false);
  //chain_bar_1.spinToPosition(60, degrees, 100, velocityUnits::pct, false);
  lift.spinToPosition(400, degrees, false);
  lift.setVelocity(45, velocityUnits::pct);
  boomerang(-25.5, -46, 1, 270, 0.4, 1500, true, 11);
  wait(200, msec);
  claw.spin(fwd, 0, volt);
  lift.spinToPosition(70, degrees, true);
  //cascade.spinToPosition(800, degrees, 100, velocityUnits::pct, true); //lower b4 score
  //chain_bar_pnuematics.set(true);//pick up pin
  /*driveTo(-10, 800, false, 9);
  turnToAngle(0, 800, true, 12);
  resetPositionBack();
  claw.spin(fwd, 12, volt);
  boomerang(-0.5, -26, 1, 0, 0.2, 900, false, 10);
  driveTo(7, 500, true, 5);
  wait(100, msec);
  lift.spinToPosition(500, degrees, false);
  boomerang(0,-5,1,0,0.2,900,true,11);
  lift.spinToPosition(0, degrees, true);
  claw.spin(fwd, 0, volt);
  wait(500, msec);

*/

  


  
}






