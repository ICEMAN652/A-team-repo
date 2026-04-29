#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>


#include "../include/autonomous.h"
#include "motor-control.h"


double field_half_size = 72.0;
// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }

//fix this function so that it can do the roller first and then move 2 inches
void movetwoinch(){
 wait(10000, msec);
 driveTo(6.5,3000);
}
//not gonna use (testing purposes)
void exampleAuton() {
 // Use this for tuning linear and turn pid
 driveTo(60, 3000);


 turnToAngle(90, 2000);
 turnToAngle(135, 2000);
 turnToAngle(150, 2000);
 turnToAngle(160, 2000);
 turnToAngle(165, 2000);
 turnToAngle(0, 2000);
 driveTo(-60, 3000);
}

//testing purposes
void exampleAuton2() {
 moveToPoint(24, 24, 1, 2000, false);
 moveToPoint(48, 48, 1, 2000, true);
 moveToPoint(24, 24, -1, 2000, true);
 moveToPoint(0, 0, 1, 2000, true);
 correct_angle = 0;
 driveTo(24, 2000, false, 8);
 turnToAngle(90, 800, false);
 turnToAngle(180, 800, true);
}


//Change the name but this would be the for the right side quadrant -- route 1
//unfinished route
void rightsidepush(){
 
}

//Change the name but this would be for the right side quadrant -- route 2
//unfinished route
void rightside4push(){
 
}


//change the name but this would be for the left side quadrant -- route 1
//unfinished route
void leftandmid(){
 
}

//change the name but this would be for the left side quadrant -- route 2
//unfinished route
void leftside7(){
 
}

//unfinished route
void sawp(){
}

//regular sawp but it also bumbs our alliance off the wall if they don't have a move two inch
//unfinished route
void sawp_bumb_version(){

}

//autonskills for override
//when done with all autonomous functions then start working on the skills code for override, until then keep the pushback code
void autonskills(){ 
  intake1.spin(fwd,12,volt);
  intake2.spin(reverse,1,volt);
  turnToAngle(60,500,true,12); // turn to face stack
  driveTo(13,750,false,8);
  intake1.spin(fwd,0,volt);
  driveTo(10,500,false,6);
  turnToAngle(-45,500,true,12); // turn to angle midgaol
  driveTo(-24,700,true,7);
  scraper.set(true);
  wait(500,msec);
  intake1.spin(reverse,12,volt); // unjam
  intake2.spin(reverse,0,volt);
  wait(100,msec);
  midgoal.set(true);
  wait(200,msec); 
  intake1.spin(fwd,10,volt); //outtake in midgoal
  wait(1000,msec);
  midgoal.set(false);
  intake2.spin(reverse,1,volt);
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
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(250, msec); 
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  resetPositionLeft();
  wait(2000, msec);
  intake2.spin(fwd,0,volt);
  moveToPoint(140, 55, 1, 1100, true, 6); // 44 to 42y , drive into matchloader point
  driveTo(9.5, 600, true, 8);
  wait(1600,msec);
  //moveToPoint(76,46,-1,1000,true,8); // move back to the long goal
  turnToAngle(87,500,true,4);
  driveTo(-50,1200,true,9); //align fully to long goal
  resetPositionLeft();
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(500, msec); //unjam
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(2000,msec); //score
  intake1.spin(fwd,0,volt);
  intake2.spin(fwd,0,volt);
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
  intake1.spin(fwd,12,volt);
  driveTo(92,2000,true,7); //drive through section
  moveToPoint(60,-28,-1,3100,true,8);
  turnToAngle(135, 400, true, 8);
  driveTo(-12.5,1000,true,9);
  
  scraper.set(true);
  intake1.spin(reverse,12,volt); // unjam
  intake2.spin(reverse,0,volt);
  wait(100,msec);
  midgoal.set(true);
  wait(200,msec); 
  intake1.spin(fwd,10,volt); //outtake in midgoal
  wait(1000,msec);
  intake1.spin(fwd, 8, volt);
  wait(500, msec);
  midgoal.set(false);
  intake2.spin(reverse,1,volt);
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
  intake1.spin(fwd, 12, volt);
  intake2.spin(reverse, -1, volt);
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
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(250, msec); 
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(1750, msec);
  intake2.spin(fwd,0,volt);
  turnToAngle(-89, 500, false, 9);
  driveTo(40, 2000, false, 7);
  wait(1600, msec);
  turnToAngle(-91, 500, true, 12);
  driveTo(-20, 1000, false, 9);
  wait(200, msec);
  turnToAngle(-90, 500, true, 12);
  driveTo(-20, 1000, false, 9);
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(250, msec); 
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(1750, msec);

  //park


  driveTo(10, 500, false, 12); // starting from long goal which haas angle of 90
  turnToAngle(55, 500, false, 12);
  intake1.spin(reverse, 12, volt);
  driveTo(-35, 1000, false, 12);
  turnToAngle(0, 500, true, 12);
  driveTo(60, 2000, true, 12);



 

 
}


