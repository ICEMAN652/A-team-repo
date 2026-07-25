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
 driveTo(12,1200,true, 12);
 wait(3000, msec);
 Brain.Screen.setCursor(4, 1);
 Brain.Screen.print("Raw R: %.1f mm | Hdg: %.1f", right_sensor.objectDistance(mm), getInertialHeading());
 Brain.Screen.setCursor(5, 1);
 Brain.Screen.print("Raw B: %.1f mm | installed: %d | detected: %d",
                     back_sensor.objectDistance(mm),
                     back_sensor.installed(),
                     back_sensor.isObjectDetected());
 distanceReset('R', 'B', 699, 1059);
 Brain.Screen.setCursor(6, 1);
 Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);
 wait(1000, msec);
 moveToPoint(0, 24, 1, 2000, true);
}
//not gonna use (testing purposes)
void exampleAuton() {
 // Use this for tuning linear and turn pid
 
 //driveTo(8, 1000, true, 12);
 //distanceReset('R', 'B', 347, 380);
 moveToPoint(0, 12, 1, 1000, false, 8);
 
 moveToPoint(0, 24, 1, 1000, true, 8);
 wait(100, msec);
 Brain.Screen.setCursor(6, 1);
 Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);
}

//testing purposes
void exampleAuton2() {
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


//Change the name but this would be the for the right side quadrant -- route 1
//unfinished route
//(-0.2,-63.7) is starting point
void rightsidepush(){
  // driveTo(8.5,500,false,12); 
  moveToPoint(0,9,1,500,false,9);
  turnToAngle(90,500,false,9); //scoring at alliance goal 
  driveTo(-25,1100,true,12); 
  wait(500, msec);
  driveTo(12,500,false,12);
  boomerang(18,34,1,50, 0.3,1000,true,9); //pickup pin and cup on the line
  wait(100, msec);
  boomerang(20,20,-1,-45, 0.2, 800, true, 9); //going to neutral goal to score
  wait(500, msec);
  moveToPoint(4,42,1,750,false,9); //go to middle pin
  turnToAngle(0, 400, false, 9);
  distanceReset('R', 'B', 1600, 1130);
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);
  boomerang(-19.65, -23.97, -1, 90, 0.3, 1000, true,11);//score middle pin
  wait(500, msec);
  boomerang(-25, 7, 1,-55 , 0.5, 1000, false,11);//pick up alliance line pin #1
  turnToAngle(90, 1000, true, 10);
  distanceReset('R', 'B', 1302, 908);
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("X: %.2f in | Y: %.2f in", x_pos, y_pos);
  moveToPoint(-18.3, 1, 1, 800, true, 12);
  //wait(500, msec);//score alliance line pin #1
  //driveTo(5, 300, false, 12);
  //boomerang(-19.2, -20.22, -1, 135, 0.4, 1000, true, 11);
//score the pre-load

// driveTo(45,1000,true,12);
//intake pin




}

//Change the name but this would be for the right side quadrant -- route 2
//unfinished route
//starting position 0.6, -65.1 180 degrees
void SAWP() {
// switch toggle 
turnToAngle(180,1200,true,12);
driveTo(19,1500,true,12);
turnToAngle(-90,1000,true,12);
driveTo(15,1000,true,12);
//outtake pre-load



}


//change the name but this would be for the left side quadrant -- route 1
//unfinished route
void leftandmid(){
turnToPoint(18.2,17.6,500,true);
moveToPoint(18.4,16.2,-1,1000,false,8);
wait(500,msec); //outtaking the pre-load into goal
turnToPoint(1.1,39.0,500,true);
driveTo(30,700,true,7);
wait(500,msec); // intaking red pin
driveTo(-28,700,true,7); 
wait(500,msec); //outtaking red pin
// turnToPoint(16.7,29.5,500,true);
moveToPoint(21.0,39.8,-1,1000,false,8); //intake yellow pin
moveToPoint(-19.1,19.6,-1,1000,false,8);
wait(500,msec); //outtake yellow pin


}

//change the name but this would be for the left side quadrant -- route 2
//unfinished route
void red_right(){
 //starting pos at -0.2, -63.7 with a heading of 0 (back to the wall)
 //scores 50 points total
 moveToPoint(-19.4, 17.1, 1, 1500, false, 12);
 turnToAngle(90, 1200, false, 12);
 //clamp
 wait(800, msec);
 //score
 boomerang(-20.8, 38.4, 1, 315, 0.3, 1300, false, 12);
 //pickup
 wait(2000, msec);
 turnToAngle(225, 800, true);
 driveTo(15, 400, true, 10);
 boomerang(-35.4, 17.1, 1, 270, 0.2, 1500, false, 12);
 driveTo(-9, 800, false, 10);
 //score
 driveTo(18, 800, false, 10);
 //pickup
 wait(1000, msec);
 driveTo(-18,800, false, 10);
 //score
 wait(600, msec);
 moveToPoint(-23.0, 8.6, 1, 1085, false, 12);
 turnToAngle(-180, 800, true);
 driveTo(6, 400, true, 10);
 //pickup
 driveTo(-13.5, 400, false, 10);
 //score 










}



//autonskills for override
//when done with all autonomous functions then start working on the skills code for override, until then keep the pushback code
void autonskills(){ 
 
  turnToAngle(60,500,true,12); // turn to face stack
  driveTo(13,750,false,8);
 
  driveTo(10,500,false,6);
  turnToAngle(-45,500,true,12); // turn to angle midgaol
  driveTo(-24,700,true,7);
  scraper.set(true);
  wait(500,msec);
  
  wait(100,msec);
  midgoal.set(true);
  wait(200,msec); 
 //outtake in midgoal
  wait(1000,msec);
  midgoal.set(false);

  driveTo(57,1500, false, 8); // drive to matchloader
  turnToAngle(-90,400,true,7); //angle to matchloader
  driveTo(23,1000,true,7); // inside of matchloader, 9 to 7 
  wait(1500,msec);
  moveToPoint(11,47,-1,1000,false,8); // hop out of matchloader, from 48 to 45 tp 49
  turnToAngle(-95,400,true,7);
  driveTo(-76,1750,false,12); //drive through corridor
  boomerang(96, 44, 1, 90, 0.3, 1000, false, 8); // align with long goal
  driveTo(-30, 1000, true, 8); //drive into it
  resetPositionLeft();
  scraper.set(true);
  
  wait(250, msec); 
 
  resetPositionLeft();
  wait(2000, msec);
  
  moveToPoint(140, 55, 1, 1100, true, 6); // 44 to 42y , drive into matchloader point
  driveTo(9.5, 600, true, 8);
  wait(1600,msec);
  //moveToPoint(76,46,-1,1000,true,8); // move back to the long goal
  turnToAngle(87,500,true,4);
  driveTo(-50,1200,true,9); //align fully to long goal
  resetPositionLeft();
  
  wait(500, msec); //unjam
  
  wait(2000,msec); //score
  
  scraper.set(false); //scraper up
  resetPositionLeft();
  moveToPoint(110,12,1,2700, true,8); // move to park
  turnToAngle(174,500,true,10); //align to park
  driveTo(18,1000,true,4); //move a lil further
  driveTo(-6, 300, true, 8);
  scraper.set(true); // scraper down
  wait(500,msec);
  driveTo(10, 750, true, 4);
  scraper.set(false); //scraper back up

  driveTo(92,2000,true,7); //drive through section
  moveToPoint(60,-28,-1,3100,true,8);
  turnToAngle(135, 400, true, 8);
  driveTo(-12.5,1000,true,9);
  
  scraper.set(true);

  wait(100,msec);
  midgoal.set(true);
  wait(200,msec); 
 
  wait(1000,msec);
 
  wait(500, msec);
  midgoal.set(false);
 
  /*driveTo(58,1500, true, 8); // drive to matchloader
  scraper.set(true);
  turnToAngle(90,400,true,7); //angle to matchloader
  driveTo(23,1000,true,7); // inside of matchloader, 9 to 7 
  wait(1500,msec);
  driveTo(-4,400,true,8);
  turnToAngle(70,500,true,12);
  driveTo(-25,700,true,12);
  turnToAngle(87,500,true,12);
  driveTo(-70,2000,true,12);*/


  scraper.set(true);
  driveTo(57,1500,true,9); //drive out
 
  turnToAngle(90,400,true,12); //face loader
  driveTo(23,700,true,8); // drive forward
  wait(100,msec);
  wait(1500,msec);
  turnToAngle(55,400,true,12);  //turn out
  wait(250,msec);
  turnToAngle(75,400,true,12); //
  driveTo(-29, 800, true, 9);
  turnToAngle(90, 400, true, 12);
  driveTo(-76,1750,false,12);//corrder
  turnToAngle(180, 600, true, 12);
  driveTo(-7, 400, true, 8);
  turnToAngle(-90, 600, true, 12);
  driveTo(-17, 1000, true, 9);
 
  wait(250, msec); 
  
  wait(1750, msec);
  
  turnToAngle(-89, 500, false, 9);
  driveTo(40, 2000, false, 7);
  wait(1600, msec);
  turnToAngle(-91, 500, true, 12);
  driveTo(-20, 1000, false, 9);
  wait(200, msec);
  turnToAngle(-90, 500, true, 12);
  driveTo(-20, 1000, false, 9);
 
  wait(250, msec); 
  
  wait(1750, msec);

  //park


  driveTo(10, 500, false, 12); // starting from long goal which haas angle of 90
  turnToAngle(55, 500, false, 12);
  
  driveTo(-35, 1000, false, 12);
  turnToAngle(0, 500, true, 12);
  driveTo(60, 2000, true, 12);



 

 
}


