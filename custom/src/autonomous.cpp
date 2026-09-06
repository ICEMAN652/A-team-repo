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
 turnToAngle(90, 1000, true, 12);
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
  intake_pnuematics.set(true);
  claw_intake.spin(fwd,12,volt);
  intake.spin(fwd,12,volt);
  boomerang(-24, -25, 1, 45, 0.3, 1500, false, 10);
  intake_pnuematics.set(false);


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
  // driveTo(8.5,500,false,12); 
  driveTo(24, 900, false, 12);
  boomerang(-17.5, 17.5, -1, 45, 0.2, 900, true);
  wait(500, msec);
  driveTo(10,500,false,12);
  boomerang(20,35,1,50, 0.2,1050,true,11); //pickup pin and cup on the line
  wait(100, msec);
  boomerang(26,19,-1,-45, 0.25, 800, true, 10); //going to neutral goal to score
  wait(500, msec);
  moveToPoint(4,42,1,750,false,10); //go to middle pin
  turnToAngle(0, 400, true, 10);
  distanceReset('L', 'B', 1680 , 1130);       // distance reset
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);
  boomerang(-19.65, -19, -1, 45, 0.3, 1000, true,11);//score middle pin
  wait(500, msec);
  boomerang(-25, 1, 1,-55 , 0.5, 900, true,11);//pick up alliance line pin #1
  turnToAngle(90, 650, true, 11);
  resetPositionRight();
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);
  moveToPoint(-40, -22, -1, 700, true, 12);//score alliance line pin #1
  wait(500, msec);
  moveToPoint(-24, -10, 1, 800, false, 12);
  boomerang(-67,-2,-1,90,0.2,1100,true,12);//going to toggle
  wait(300, msec);//toggle
  resetPositionBack();
  driveTo(0.5, 100, false, 12);
  turnToAngle(184, 600, true, 12);
  resetPositionRight();
  driveTo(10, 300, false, 12);
  moveToPoint(-50, -44, 1, 800, true, 12);//going to pick up second alliance pin
  moveToPoint(-47, -26, -1, 900, true, 12);//going to scoring alliance pin
  wait(500, msec);// score alliance pill 
  
  
}






