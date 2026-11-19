#include "vex.h"
#include "utils.h"
#include <cmath>
#include <cstdio>
#include "motor-control.h"
#include "../include/field-resets.h"

// Team-specific position resets, moved here from src/motor-control.cpp so RW-Template updates keep them.


// ============================================================================
// ANGLE-CORRECTED DISTANCE RESET
// ============================================================================

static float dr_mmToIn(float mm)                    { return mm / 25.4f; }
static double dr_sensorDistMm(DistResetSensor* s)    { return s->sensor->objectDistance(mm); }

/*
 * distanceReset
 * Reads the raw sensor-lens-to-wall distance off each of the two chosen
 * sensors and sets x_pos/y_pos to the ERROR between the distance you expected
 * (expectedXmm, expectedYmm, measured/CAD'd by hand beforehand) and what the
 * sensor actually reads right now. No heading correction or field-center
 * projection is applied -- this is a direct raw-distance comparison, not a
 * field-position calculation.
 *
 * xDirection / yDirection: which sensor is pointed toward that wall.
 *   'F' = front, 'B' = back, 'R' = right, 'L' = left
 *
 * expectedXmm / expectedYmm are in millimeters (matching the raw distance
 * sensor units) -- converted to inches internally since x_pos/y_pos work in
 * inches like the rest of this file.
 *
 * Sensor layout is built from the existing robot-config sensors (one sensor per side).
 */
void distanceReset(char xDirection, char yDirection, double expectedXmm, double expectedYmm) {
    // Build sensor descriptors from existing robot-config sensors.
    DistResetSensor drFront = { &front_sensor, front_sensor_offsetX, front_sensor_offsetY, 0.0   };
    DistResetSensor drRight = { &right_sensor, right_sensor_offsetX, right_sensor_offsetY, 90.0  };
    DistResetSensor drBack  = { &back_sensor,  back_sensor_offsetX,  back_sensor_offsetY,  180.0 };
    DistResetSensor drLeft  = { &left_sensor,  left_sensor_offsetX,  left_sensor_offsetY,  270.0 };

    auto pick = [&](char dir) -> DistResetSensor* {
        switch (dir) {
            case 'F': return &drFront;
            case 'R': return &drRight;
            case 'B': return &drBack;
            case 'L': return &drLeft;
            default:  return nullptr;
        }
    };

    DistResetSensor* side  = pick(xDirection);
    DistResetSensor* front = pick(yDirection);

    double rawXIn = (side  != nullptr && side->sensor  != nullptr) ? dr_mmToIn((float)dr_sensorDistMm(side))  : NAN;
    double rawYIn = (front != nullptr && front->sensor != nullptr) ? dr_mmToIn((float)dr_sensorDistMm(front)) : NAN;

    if (std::isnan(rawXIn) || std::isnan(rawYIn) || rawXIn > 300 || rawYIn > 300) {
        Brain.Screen.print("distanceReset: bad sensors, aborting");
        return;
    }

    double expectedX = dr_mmToIn((float)expectedXmm);
    double expectedY = dr_mmToIn((float)expectedYmm);

    x_pos = rawXIn - expectedX;
    y_pos = rawYIn - expectedY;

    char buf[64];
    snprintf(buf, sizeof(buf), "%.3f, %.3f, %.3f", (float)x_pos, (float)y_pos, (float)getInertialHeading());
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Distance Reset: %s", buf);
}
