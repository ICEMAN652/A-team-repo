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

/*
 * distanceReset
 * Resets x_pos and y_pos from two distance sensors: one on the SIDE of the robot
 * (sideSensor = 'L' or 'R') and one on the FRONT or BACK (endSensor = 'F' or 'B').
 *
 * The robot's heading decides which wall each sensor is looking at, so you never
 * have to work out "is this the x wall or the y wall":
 *   - The heading is rounded to the nearest square angle (0, 90, 180, 270), so 270.45,
 *     271 or 268 all count as 270. Headings past 360 or below 0 (e.g. -90, 450) work too.
 *   - If the robot is more than dr_max_off_square_deg away from square, the reset is
 *     skipped (the readings would be too far off).
 *   - The small angle that's left (e.g. 0.45 deg) is corrected for: a tilted sensor reads
 *     a little long, so the reading is multiplied by cos(angle off square).
 *   - Each sensor then sets x if it faces a side wall (+x/-x) or y if it faces an end wall
 *     (+y/-y), with the sign flipped when it faces a + wall, so that moving toward +x/+y
 *     always makes x/y bigger, the same as odometry.
 *
 * Field directions (same as the rest of the template): 0 deg faces +y, clockwise positive,
 * 90 deg faces +x.
 *
 *   heading   sideSensor L/R sets      endSensor F/B sets
 *   0         x                        y
 *   90        y                        x
 *   180       x                        y
 *   270       y                        x
 *
 * expectedSideMm / expectedEndMm: what THAT sensor reads (millimeters, lens to wall) when the
 * robot is at the spot you want to be 0. So the position afterwards is measured from that spot:
 * at the spot x = 0 / y = 0, and it grows toward +x / +y like normal odometry.
 *
 * Returns true if the position was reset. If info is given, it's filled in with what was used
 * (the debug screen uses this; auton code can leave it out).
 */
static const double dr_max_off_square_deg = 15;   // skip the reset if the robot is more crooked than this
static const double dr_max_reading_mm = 2000;     // the V5 distance sensor reads up to ~2000 mm (9999 = sees nothing)

static double dr_wrap360(double deg) {
    double wrapped = fmod(deg, 360.0);
    return wrapped < 0 ? wrapped + 360.0 : wrapped;
}

// Which wall a sensor faces once the heading is squared up: 0 -> +y, 90 -> +x, 180 -> -y, 270 -> -x.
// Sets axis to 'x' or 'y', and sign to +1 (faces a - wall) or -1 (faces a + wall).
static void dr_wallFor(double facing_deg, char& axis, int& sign) {
    int facing = (int)lround(dr_wrap360(facing_deg)) % 360;
    switch (facing) {
        case 0:   axis = 'y'; sign = -1; break;   // +y wall
        case 90:  axis = 'x'; sign = -1; break;   // +x wall
        case 180: axis = 'y'; sign = +1; break;   // -y wall
        default:  axis = 'x'; sign = +1; break;   // 270: -x wall
    }
}

bool distanceReset(char sideSensor, char endSensor, double expectedSideMm, double expectedEndMm, DistanceResetInfo* info) {
    DistanceResetInfo local;
    DistanceResetInfo& r = info ? *info : local;
    r = DistanceResetInfo();

    DistResetSensor drFront = { &front_sensor, front_sensor_offsetX, front_sensor_offsetY, 0.0   };
    DistResetSensor drRight = { &right_sensor, right_sensor_offsetX, right_sensor_offsetY, 90.0  };
    DistResetSensor drBack  = { &back_sensor,  back_sensor_offsetX,  back_sensor_offsetY,  180.0 };
    DistResetSensor drLeft  = { &left_sensor,  left_sensor_offsetX,  left_sensor_offsetY,  270.0 };

    // Side sensor must be L/R and end sensor F/B, so the two always cover one x wall and one y wall
    DistResetSensor* side = sideSensor == 'L' ? &drLeft  : sideSensor == 'R' ? &drRight : nullptr;
    DistResetSensor* end  = endSensor  == 'F' ? &drFront : endSensor  == 'B' ? &drBack  : nullptr;
    if (side == nullptr || end == nullptr) {
        r.fail_reason = "use L/R first, then F/B";
        printf("distanceReset: %s (got '%c', '%c')\n", r.fail_reason, sideSensor, endSensor);
        return false;
    }

    // Square up the heading: 270.45 -> 270, off_square = 0.45
    double heading = dr_wrap360(getInertialHeading());
    r.square_heading = dr_wrap360(round(heading / 90.0) * 90.0);
    r.off_square_deg = heading - r.square_heading;
    if (r.off_square_deg > 180) r.off_square_deg -= 360;    // e.g. 359.6 -> -0.4 off 0
    if (r.off_square_deg < -180) r.off_square_deg += 360;
    if (fabs(r.off_square_deg) > dr_max_off_square_deg) {
        r.fail_reason = "robot not square";
        printf("distanceReset: %s (heading %.1f)\n", r.fail_reason, heading);
        return false;
    }

    r.side_mm = side->sensor->objectDistance(mm);
    r.end_mm  = end->sensor->objectDistance(mm);
    if (!(r.side_mm > 0 && r.side_mm <= dr_max_reading_mm) || !(r.end_mm > 0 && r.end_mm <= dr_max_reading_mm)) {
        r.fail_reason = "sensor sees no wall";
        Brain.Screen.print("distanceReset: bad sensors, aborting");
        printf("distanceReset: %s (%c=%.0fmm %c=%.0fmm)\n", r.fail_reason, sideSensor, r.side_mm, endSensor, r.end_mm);
        return false;
    }

    // Which wall each sensor faces at this heading
    dr_wallFor(r.square_heading + side->headingOffsetDeg, r.side_axis, r.side_sign);
    dr_wallFor(r.square_heading + end->headingOffsetDeg,  r.end_axis,  r.end_sign);

    // Tilted sensors read long: straight-on distance = reading * cos(angle off square)
    double tilt = cos(r.off_square_deg * M_PI / 180.0);
    double side_pos = r.side_sign * dr_mmToIn((float)(r.side_mm * tilt - expectedSideMm));
    double end_pos  = r.end_sign  * dr_mmToIn((float)(r.end_mm  * tilt - expectedEndMm));

    if (r.side_axis == 'x') { x_pos = side_pos; y_pos = end_pos; }
    else                    { y_pos = side_pos; x_pos = end_pos; }
    r.ok = true;

    char buf[64];
    snprintf(buf, sizeof(buf), "%.3f, %.3f, %.3f", (float)x_pos, (float)y_pos, (float)getInertialHeading());
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Distance Reset: %s", buf);
    return true;
}
