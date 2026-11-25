#pragma once
#include <cmath>

// Team-specific position resets. These live in custom/ so RW-Template updates don't overwrite them.

// Sensor descriptor for distanceReset - configure offsetX/offsetY per your robot geometry.
// offsetX: lateral offset from robot center, perpendicular to sensor pointing direction (inches)
// offsetY: distance from robot center to sensor face, along sensor pointing direction (inches)
// headingOffsetDeg: which way this sensor points relative to robot-forward
//   (0 = front, 90 = right, 180 = back, 270 = left)
struct DistResetSensor {
    vex::distance* sensor;  // nullptr if this sensor position is not installed
    double offsetX;
    double offsetY;
    double headingOffsetDeg;
};

// What distanceReset() used, for debugging (see chkDistanceReset in auton-debug).
struct DistanceResetInfo {
    bool ok = false;
    const char* fail_reason = "";   // why it was skipped, when ok is false
    double square_heading = 0;      // heading rounded to 0/90/180/270
    double off_square_deg = 0;      // how far the real heading was from that (e.g. 0.45)
    double side_mm = 0, end_mm = 0; // raw readings
    char side_axis = '?', end_axis = '?';  // 'x' or 'y': which coordinate each sensor set
    int side_sign = 0, end_sign = 0;       // +1 = sensor faces a - wall, -1 = faces a + wall
};

// Resets x_pos/y_pos from two distance sensors. The heading decides which sensor sets x and
// which sets y (and the sign), so you only pick the sensors:
//   sideSensor: 'L' or 'R'      endSensor: 'F' or 'B'
//   expectedSideMm / expectedEndMm: what THAT sensor reads (mm) at the spot you want to be 0.
// Works with headings that aren't exactly square (270.45, 271, ...): it rounds to the nearest
// 0/90/180/270 and corrects the reading for the small tilt. Skipped if more than 15 deg off square.
// Example, robot at ~180:  distanceReset('L', 'F', 377, 690);
// Returns true if the position was reset. Details of the full rules: custom/src/field-resets.cpp
bool distanceReset(char sideSensor, char endSensor, double expectedSideMm, double expectedEndMm,
                   DistanceResetInfo* info = nullptr);
