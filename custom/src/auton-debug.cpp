#include "vex.h"
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "../include/robot-config.h"
#include "motor-control.h"
#include "../include/field-resets.h"
#include "../include/auton-debug.h"

// Auton debug screen + checks. How to use them and what the screen shows: custom/include/auton-debug.h

extern motor lift_3;   // defined in robot-config.cpp (not in the header)

bool debug_checkpoints = true;
static double auton_start_ms = 0;
static const char* last_label = "start";
static const char* auton_title = "AUTON";
static bool debug_thread_started = false;

static double autonSec() { return (Brain.timer(msec) - auton_start_ms) / 1000.0; }

// ---- scrolling log ----
static const int log_max = 13;                 // mono15 font: rows 3-15
static char log_text[log_max][60];
static const vex::color* log_color[log_max];
static int log_count = 0;

static void addLogText(const vex::color& color, const char* text) {
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

static void addLog(const vex::color& color, const char* format, ...) {
  char text[60];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  addLogText(color, text);
}

void debugLog(const vex::color& color, const char* format, ...) {
  if (!debug_checkpoints) return;
  char text[60];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  addLogText(color, text);
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
  Brain.Screen.print("%s  t%.1f  last: %s", auton_title, autonSec(), last_label);
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

void startAutonDebug(const char* auton_name) {
  auton_start_ms = Brain.timer(msec);
  auton_title = auton_name;
  last_label = "start";
  log_count = 0;
  wire_faults[0] = 0;
  wire_fault_count = 0;
  imu_nan_reported = false;
  for (int i = 0; i < wire_count; i++) wires[i].was_plugged = true;
  Brain.Screen.clearScreen();
  checkpoint("start"); // if you don't see this on the Brain screen, the new code isn't on the robot
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

// Two lines: which sensor set which axis at what heading, then readings and old>new position, e.g.
//   DR OK h270 (+0.4 off) R>y F>x
//     R412/410 F151/150 x31.0>0.1 y-16.0>0.5        (reading/expected mm)
bool chkDistanceReset(char side_sensor, char end_sensor, double expected_side_mm, double expected_end_mm) {
  double old_x = x_pos, old_y = y_pos;
  DistanceResetInfo info;
  bool ok = distanceReset(side_sensor, end_sensor, expected_side_mm, expected_end_mm, &info);
  if (!debug_checkpoints) return ok;
  if (ok) {
    addLog(green, "DR OK h%.0f (%+.1f off) %c>%c %c>%c", info.square_heading, info.off_square_deg,
           side_sensor, info.side_axis, end_sensor, info.end_axis);
    addLog(green, "  %c%.0f/%.0f %c%.0f/%.0f x%.1f>%.1f y%.1f>%.1f", side_sensor, info.side_mm, expected_side_mm,
           end_sensor, info.end_mm, expected_end_mm, old_x, x_pos, old_y, y_pos);
  } else {
    addLog(red, "DR SKIP %s h%.1f %c=%.0f %c=%.0f", info.fail_reason, getInertialHeading(),
           side_sensor, info.side_mm, end_sensor, info.end_mm);
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
