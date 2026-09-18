#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>


#include "../include/autonomous.h"
#include "../include/robot-config.h"
#include "motor-control.h"




// AprilTag helpers now in motor-control.cpp, alongside the other position-tracking/reset functions


// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }





//fix this function so that it can do the roller first and then move 2 inches
void movetwoinch(){
  //driveTo(24, 1500, true, 12);
  turnToAngle(90, 1200, true, 12);
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
  driveTo(558, 500, false, 12);
  turnToAngle(180, 1000, false, 12);
  driveTo(15, 1000, false, 9);
  driveTo(-5, 500, false, 10);
  driveTo(5, 800, false, 8);
  driveTo(-5, 300, false, 12);
  resetPositionFront();
  boomerang(-24, -25, 1, 45, 0.3, 1500, false, 10);


  /*driveTo(1.8, 1000, true, 3);
  wait(3000, msec);
  driveTo(-3.5, 1000, true, 8);*/
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

//input starting point
void BlueLeft(){
  resetPositionBack();
  driveTo(18,1200,false,12);
  boomerang(-23,-29,1,-45,0.3,1000,true,10);
  wait(200, msec);
  boomerang(-24,-46.5,-1,45,0.3,550,true,10);
  wait(500, msec);
  boomerang(18,-25,1,45,0.3,1000,true,10);
  boomerang(18, -41, -1, 0, 0.4, 1000, true, 10);
  wait(500, msec);
  resetPositionRight();
  boomerang(46,-46,1,135,0.3,1000,true,10);
  turnToAngle(90,400, true);
  moveToPoint(25,-46,-1,500,true,10);
  resetPositionRight();
  wait(500, msec);
  driveTo(13,500,true, 10);
  turnToAngle(229, 600, true);
  driveTo(7, 500 ,false,9);
  driveChassis(4.5, 6);
  wait(250, msec);
  driveChassis(0,0);
  wait(500,msec);
  turnToAngle(180,600,true);
  resetPositionLeft();
  moveToPoint(22,-49,-1,600,true,10);
  // wait(250, msec);
  // boomerang(24,-49.5,-1,135,0.3,1000,true,10);
  /*turnToAngle(90,500,true);
  //heading at 90 here
  driveTo(-20,500, true, 10);
  wait(500, msec);
  boomerang(-25.5,-23,1,180,0.3,800,true,8);
  //driveTo(3,500, true, 5);
  //wait(300, msec);*/
  //moveToPoint(0,-22.5,-1,1000,true,9);
  
  
}

//unfinished routmoveToPoint(emoveToPoint(
//(-0.2,-63.7) is starting point
void SAWP(){
  resetPositionBack();
  //resetPositionLeft();
  //moveToPoint(0, -6, 1, 400, true, 12);
  driveTo(-6, 400, true, 12);//toggle #1
  driveTo(6, 300, true, 12);
  driveTo(-9, 450, true, 12);//toggle #2
  driveTo(5, 400, true, 12);
  resetPositionBack();

  chain_bar_1.spinToPosition(55, degrees, 80, velocityUnits::pct, false);
  boomerang(20.5, -49.25, 1, 90, 0.4, 1200, true, 7);//drive to alliance goal
  resetPositionRight();
  wait(400, msec);
  chain_bar_pnuematics.set(true);//drop preload in alliance goal
  wait(300, msec);
  driveTo(-10, 800, true, 12);//back out of allaince goal
  chain_bar_1.spinToPosition(0, degrees, 80, velocityUnits::pct, false);
  boomerang(19.4, -29.5, 1, 30, 0.3, 1100, true, 7);//drive to the first pin stack and align
  driveTo(6, 750, false, 3);//drive into it
  //wait(100, msec);
  chain_bar_pnuematics.set(false);//got the first pin stack on allaiance line
  driveTo(1.7, 250, true, 3);//drive into it
  wait(300, msec);
  cascade.spinToPosition(1220, degrees, 90, velocityUnits::pct, false);
  wait(300, msec);
  turnToAngle(135, 1000, false, 9);
  boomerang(23, -47, 1, 180, 0.4, 1000, true, 7);//drive back to allaicne goal
  wait(100, msec);
  cascade.spinToPosition(550, degrees, 90, velocityUnits::pct, true); //lower b4 score
  //chain_bar_1.spinToPosition(100, degrees, 80, velocityUnits::pct, false);
  chain_bar_pnuematics.set(true);//score the pin stack on allaince goal
  wait(70, msec);
  cascade.spinToPosition(1220, degrees, 90, velocityUnits::pct, false);
  wait(300, msec);

  driveTo(-3, 200, false, 12);
  cascade.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  chain_bar_1.spinToPosition(0, degrees, 90, velocityUnits::pct, false);
  turnToAngle(180, 200, true, 12);
  resetPositionLeft();
  driveTo(-5, 350, false, 12);
  boomerang(44, -44, 1, 134, 0.3, 1000, true, 8);
  driveTo(1, 700, false, 2.5);//drive into it
  chain_bar_pnuematics.set(false);//pick up pin
  driveTo(0.5, 200, true, 2);//drive into it
  wait(100, msec);
  cascade.spinToPosition(800, degrees, 90, velocityUnits::pct, false);
  moveToPoint(27, -23, -1, 800, true, 12);
  moveToPoint(-18, -60, -1, 1000, true, 9);
  turnToAngle(90, 200, true, 12);
  //boomerang(12, -55, -1, -180, 1, 1000, true, 12);

  


  
}






