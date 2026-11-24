#pragma once
#include "vex.h"

// Auton debug screen + checks (custom/src/auton-debug.cpp). Use from any auton:
//
//   void myAuton() {
//     startAutonDebug("MY AUTON");          // first line of the auton
//     ...
//     chkDriveTo(-8, 600, true, 12);        // driveTo that reports if it didn't move
//     chkResetLeft();                       // resetPositionLeft that reports what it did
//     checkpoint("scored");                 // log position/heading/lift/time here
//   }
//
// Brain screen while it runs:
//   Row 1 (green):  auton name, auton time, last checkpoint name
//   Row 2:          WIRES - "all plugged in", or RED with every device that came
//                   unplugged during the run
//   Rows 3+:        log, newest at the bottom (yellow):
//     white   checkpoint   name x y heading lift-deg t(sec)
//     green   reset OK     rstL OK 380mm x31.0>30.2 y-16.0>-16.0   (old>new position)
//     red     reset SKIP   rstL SKIP nothing seen   / rstL SKIP 400mm (angle/jump)
//     red     driveTo FAIL drv-8 FAIL 600ms moved 0.1/8.0 V0.0/0.0 h180 hold180
//     red     UNPLUGGED IMU P5 t41.8 after mtp1
// Every line is also printed to the USB terminal as "[CP] ...".

// false = turn all of the debug screen/logging off (the chk* functions still do their move/reset)
extern bool debug_checkpoints;

// Call as the first line of an auton: restarts the timer, clears the log and wire faults,
// starts the background wire check / screen refresh, and logs a "start" checkpoint.
void startAutonDebug(const char* auton_name);

// Logs one line: label, x, y, heading, lift degrees, seconds since startAutonDebug().
void checkpoint(const char* label);

// Adds your own line to the log, printf style, e.g. debugLog(vex::red, "claw stuck %d", n);
// Use a named color (vex::red, vex::green, vex::yellow, vex::white, ...), not vex::color(...).
void debugLog(const vex::color& color, const char* format, ...);

// Same as resetPositionLeft/Right/Back/Front, but logs whether it reset and old>new position.
bool chkResetLeft();
bool chkResetRight();
bool chkResetBack();
bool chkResetFront();

// Same as distanceReset (field-resets.h): side sensor 'L'/'R' first, then 'F'/'B'.
// Logs which sensor set x/y at what heading, the raw/expected readings and old>new position.
bool chkDistanceReset(char side_sensor, char end_sensor, double expected_side_mm, double expected_end_mm);

// Same as driveTo, but logs a red FAIL line if the robot drove less than half the distance:
//   ~0 ms             -> driveTo quit immediately
//   V near 0          -> driveTo never sent power (heading/IMU problem, or motors unplugged)
//   V high, moved ~0  -> power was sent but wheels didn't turn (motor/wire/encoder problem)
//   hold              -> the heading driveTo tries to keep (set by the last turn)
void chkDriveTo(double distance_in, double time_limit_msec, bool exit, double max_output);
