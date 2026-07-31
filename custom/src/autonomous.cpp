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
 driveTo(12,1200,false, 12);
 turnToAngle(45, 800, true, 12);
 wait(100, msec);
 resetPositionBack();
 Brain.Screen.setCursor(6, 1);
 Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);
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

//(-0.2,-63.7) is starting point with the toggle on the back
void RedRight() {
  resetPositionBack();
  driveTo(18,1200,false,12);
  boomerang(20,-25,-1,98,0.2,2000,false,5);
  moveToPoint(0,-36,1,1000,false,11);
  boomerang(20,-48,-1,-45,0.2,950,false,9);//score 1st
  moveToPoint(-15,-30,1,1200,true,10);
  turnToAngle(137,800,true);
  driveTo(-3, 500, true, 9);
  moveToPoint(-5,-48,1,1000,false,10);
  boomerang(-22,-51,-1,90,0.3,2500,false,10);
  wait(500, msec); //score 2nd
  // resetPositionRight();
  // Brain.Screen.setCursor(6, 1);
  // Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);
  driveTo(20,500,true,9);


}

//input starting point
void RedLeft(){
  //unfinished route
}

//unfinished route
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
  distanceReset('L', 'B', 1680 , 1130);
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

  

  /*driveTo(6, 400, false, 12);
  boomerang(-50, -46, 1, 225 , 0.3, 1000, true,11);
  moveToPoint(-51,-26,-1,700,true,12);//score second alliance pin
  wait(500, msec);
  driveTo(4, 300, false, 12);
  turnToAngle(225, 700, false, 12);
  boomerang(-65,-3,-1,90,0.6,1500, false, 12);
  driveTo(-1, 700, true, 12);*/
  //alt route 2 below
  //moveToPoint(-30,-29,1,800,true,12);
  //turnToAngle(135, 800, false, 11);
  //boomerang(-73, -7, -1, 90 , 0.2, 1500, true,12);
  

  
  
  //score the pre-load

  // driveTo(45,1000,true,12);
  //intake pin
}






