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

// Reads the raw sensor-lens-to-wall distance off each of two chosen distance
// sensors and sets x_pos/y_pos to the ERROR between the distance you expected
// (expectedXmm, expectedYmm) and what the sensor actually reads right now --
// a direct raw-distance comparison, with no heading correction or field-center
// projection applied. Does nothing if either sensor can't see a wall.
// xDirection: which sensor to use for the X-side reading ('F'=front, 'B'=back, 'R'=right, 'L'=left)
// yDirection: which sensor to use for the Y-side reading ('F'=front, 'B'=back, 'R'=right, 'L'=left)
// expectedXmm / expectedYmm: the sensor-to-wall distance you expect right now,
//   in MILLIMETERS (measured by hand beforehand) -- converted to inches
//   internally to match x_pos/y_pos, which stay in inches like the rest of
//   the template.
void distanceReset(char xDirection, char yDirection, double expectedXmm, double expectedYmm);
