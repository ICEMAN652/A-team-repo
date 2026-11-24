#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>
#include <cstdarg>
#include <cstdio>
#include <cstring>


#include "../include/autonomous.h"
#include "../include/robot-config.h"
#include "motor-control.h"
#include "../include/field-resets.h"
#include "../include/lift.h"

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
// ---------------------------------------------------------------------------
// SKILLS DEBUG SCREEN (Brain)
//   Row 1 (green):  header, auton time, last checkpoint name
//   Row 2:          WIRES - green "all plugged in", or RED with every device that came
//                   unplugged during the run, when, and after which checkpoint
//   Rows 3+:        log, newest at the bottom (yellow):
//     white   checkpoint   name x y heading lift-deg t(sec)
//     green   reset OK     rstL OK 380mm x31.0>30.2 y-16.0>-16.0   (old>new position)
//     red     reset SKIP   rstL SKIP nothing seen   / rstL SKIP 400mm (bad angle/jump)
//     red     driveTo FAIL drv-8 FAIL 600ms moved 0.1/8.0 V0.0/0.0 h180 hold180
// Every line is also printed to the USB terminal as "[CP] ...".
// Set debug_checkpoints to false to turn all of this off.
// ---------------------------------------------------------------------------
extern motor lift_3;   // defined in robot-config.cpp (not in the header)

static bool debug_checkpoints = true;
static double auton_start_ms = 0;
static const char* last_label = "start";
static bool debug_thread_started = false;

static double autonSec() { return (Brain.timer(msec) - auton_start_ms) / 1000.0; }

// ---- scrolling log ----
static const int log_max = 13;                 // mono15 font: rows 3-15
static char log_text[log_max][60];
static const vex::color* log_color[log_max];
static int log_count = 0;

static void addLog(const vex::color& color, const char* format, ...) {
  char text[60];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  if (log_count == log_max) {                  // oldest scrolls off the top
    for (int i = 1; i < log_max; i++) {
      snprintf(log_text[i - 1], 60, "%s", log_text[i]);
      log_color[i - 1] = log_color[i];
    }
    log_count = log_max - 1;
  }
  snprintf(log_text[log_count], 60, "%s", text);
  log_color[log_count] = &color;
  log_count++;
  printf("[CP] %s\n", text);
}

// ---- wire check: every device the route uses ----
struct WireCheck { const char* name; bool (*plugged)(); bool was_plugged; };
static WireCheck wires[] = {
  {"Ldrive P20", [] { return left_chassis1.installed(); }, true},
  {"Ldrive P18", [] { return left_chassis2.installed(); }, true},
  {"Rdrive P16", [] { return right_chassis1.installed(); }, true},
  {"Rdrive P9",  [] { return right_chassis2.installed(); }, true},
  {"IMU P5",     [] { return inertial_sensor.installed(); }, true},
  {"vTrack P17", [] { return vertical_tracker.installed(); }, true},
  {"lift P3",    [] { return lift_1.installed(); }, true},
  {"lift P14",   [] { return lift_2.installed(); }, true},
  {"lift P8",    [] { return lift_3.installed(); }, true},
  {"claw P1",    [] { return claw.installed(); }, true},
  {"intake P13", [] { return active_toggle.installed(); }, true},
  {"distL P10",  [] { return left_sensor.installed(); }, true},
  {"distR P19",  [] { return right_sensor.installed(); }, true},
  {"distB P4",   [] { return back_sensor.installed(); }, true},
  {"distF P15",  [] { return front_sensor.installed(); }, true},
};
static const int wire_count = sizeof(wires) / sizeof(wires[0]);
static char wire_faults[60] = "";              // pinned on row 2
static int wire_fault_count = 0;
static bool imu_nan_reported = false;

static void checkWires() {
  for (int i = 0; i < wire_count; i++) {
    bool plugged = wires[i].plugged();
    if (!plugged && wires[i].was_plugged) {
      wire_fault_count++;
      addLog(red, "UNPLUGGED %s t%.1f after %s", wires[i].name, autonSec(), last_label);
      size_t used = strlen(wire_faults);
      snprintf(wire_faults + used, sizeof(wire_faults) - used, "%s%s", used ? ", " : "", wires[i].name);
    } else if (plugged && !wires[i].was_plugged) {
      addLog(yellow, "reconnected %s t%.1f", wires[i].name, autonSec());
    }
    wires[i].was_plugged = plugged;
  }
  if (!imu_nan_reported && std::isnan(getInertialHeading())) {
    imu_nan_reported = true;
    addLog(red, "IMU heading is NaN t%.1f after %s", autonSec(), last_label);
  }
}

// ---- drawing ----
static void drawDebugScreen() {
  Brain.Screen.setFont(mono15);
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(green);
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("SKILLS DEBUG  t%.1f  last: %s", autonSec(), last_label);
  Brain.Screen.setCursor(2, 1);
  if (wire_fault_count == 0) {
    Brain.Screen.print("WIRES: all plugged in");
  } else {
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("WIRES OUT: %s", wire_faults);
  }
  for (int i = 0; i < log_count; i++) {
    bool newest = i == log_count - 1;
    const vex::color& c = (newest && log_color[i] == &white) ? yellow : *log_color[i];
    Brain.Screen.setPenColor(c);
    Brain.Screen.setCursor(i + 3, 1);
    Brain.Screen.print("%s", log_text[i]);
  }
  Brain.Screen.setPenColor(white);
}

// ---- background loop: wire checks, driveTo voltage sampling, screen refresh ----
static volatile bool drive_sampling = false;
static double drive_max_left_v = 0, drive_max_right_v = 0;

static int debugLoop() {
  int tick = 0;
  while (true) {
    if (drive_sampling) {
      drive_max_left_v  = fmax(drive_max_left_v,  fabs(left_chassis1.voltage(voltageUnits::volt)));
      drive_max_right_v = fmax(drive_max_right_v, fabs(right_chassis1.voltage(voltageUnits::volt)));
    }
    if (tick % 10 == 0) checkWires();          // every 100 ms
    if (tick % 25 == 0) drawDebugScreen();     // every 250 ms
    tick++;
    wait(10, msec);
  }
  return 0;
}

static void startDebug() {
  if (!debug_checkpoints || debug_thread_started) return;
  debug_thread_started = true;
  vex::thread debug_thread(debugLoop);
}

void checkpoint(const char* label) {
  if (!debug_checkpoints) return;
  startDebug();
  last_label = label;
  addLog(white, "%s x%.1f y%.1f h%.0f L%.0f t%.1f", label, (float)x_pos, (float)y_pos,
         (float)getInertialHeading(), (float)lift.position(degrees), (float)autonSec());
  drawDebugScreen();
}

// ---- distance reset checks: show whether it reset, the reading, and old > new position ----
static bool chkReset(const char* name, bool (*reset)(), distance& sensor) {
  double old_x = x_pos, old_y = y_pos;
  bool seen = sensor.isObjectDetected();
  double reading_mm = sensor.objectDistance(mm);
  bool ok = reset();
  if (!debug_checkpoints) return ok;
  if (ok) {
    addLog(green, "%s OK %.0fmm x%.1f>%.1f y%.1f>%.1f", name, reading_mm, old_x, x_pos, old_y, y_pos);
  } else if (!seen) {
    addLog(red, "%s SKIP nothing seen x%.1f y%.1f", name, x_pos, y_pos);
  } else {
    addLog(red, "%s SKIP %.0fmm (angle/jump) x%.1f y%.1f", name, reading_mm, x_pos, y_pos);
  }
  drawDebugScreen();
  return ok;
}
bool chkResetLeft()  { return chkReset("rstL", resetPositionLeft,  left_sensor); }
bool chkResetRight() { return chkReset("rstR", resetPositionRight, right_sensor); }
bool chkResetBack()  { return chkReset("rstB", resetPositionBack,  back_sensor); }
bool chkResetFront() { return chkReset("rstF", resetPositionFront, front_sensor); }

static distance& sensorFor(char dir) {
  switch (dir) {
    case 'F': return front_sensor;
    case 'B': return back_sensor;
    case 'R': return right_sensor;
    default:  return left_sensor;
  }
}

bool chkDistanceReset(char x_dir, char y_dir, double expected_x_mm, double expected_y_mm) {
  double old_x = x_pos, old_y = y_pos;
  double raw_x = sensorFor(x_dir).objectDistance(mm), raw_y = sensorFor(y_dir).objectDistance(mm);
  bool ok = distanceReset(x_dir, y_dir, expected_x_mm, expected_y_mm);
  if (!debug_checkpoints) return ok;
  if (ok) {
    addLog(green, "DR OK %c%.0f/%.0f %c%.0f/%.0f x%.1f>%.1f y%.1f>%.1f", x_dir, raw_x, expected_x_mm,
           y_dir, raw_y, expected_y_mm, old_x, x_pos, old_y, y_pos);
  } else {
    addLog(red, "DR SKIP %c=%.0fmm %c=%.0fmm (9999=sees nothing)", x_dir, raw_x, y_dir, raw_y);
  }
  drawDebugScreen();
  return ok;
}

// ---- driveTo check: did it actually move? ----
// "moved a/b" = inches driven / inches asked. "V l/r" = highest voltage the drive motors got.
//   ~0 ms         -> driveTo quit immediately
//   V near 0      -> driveTo never sent power (heading/IMU problem, or motors unplugged)
//   V high, moved ~0 -> power was sent but wheels didn't turn (motor/wire/encoder problem)
// "hold" = the heading driveTo tries to hold (set by the last turn).
void chkDriveTo(double distance_in, double time_limit_msec, bool exit, double max_output) {
  double start_left = left_chassis.position(degrees), start_right = right_chassis.position(degrees);
  double start_ms = Brain.timer(msec);
  float hold = (float)correct_angle;
  drive_max_left_v = drive_max_right_v = 0;
  drive_sampling = true;
  driveTo(distance_in, time_limit_msec, exit, max_output);
  drive_sampling = false;
  if (!debug_checkpoints) return;
  double took_ms = Brain.timer(msec) - start_ms;
  double moved = (fabs(left_chassis.position(degrees) - start_left) + fabs(right_chassis.position(degrees) - start_right))
                 / 2.0 / 360.0 * wheel_distance_in;
  bool ok = moved >= fabs(distance_in) * 0.5;
  char text[60];
  snprintf(text, sizeof(text), "drv%.0f %s %.0fms moved %.1f/%.1f V%.1f/%.1f h%.0f hold%.0f", distance_in,
           ok ? "ok" : "FAIL", took_ms, moved, fabs(distance_in), drive_max_left_v, drive_max_right_v,
           (float)getInertialHeading(), hold);
  if (ok) {
    printf("[CP] %s\n", text);                 // terminal only, keeps the screen for problems
  } else {
    addLog(red, "%s", text);
    drawDebugScreen();
  }
}

void near_side_auton(){
  auton_start_ms = Brain.timer(msec);
  Brain.Screen.clearScreen();
  checkpoint("start"); // if you don't see this on the Brain screen, the new code isn't on the robot
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
  auton_start_ms = Brain.timer(msec);
  Brain.Screen.clearScreen();
  checkpoint("start"); // if you don't see this on the Brain screen, the new code isn't on the robot
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





