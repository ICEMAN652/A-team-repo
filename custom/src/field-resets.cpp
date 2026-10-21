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

// Rotates a local (forward, right) displacement into field-aligned (x, y)
// axes at the given heading -- same forward/right convention used throughout
// src/motor-control.cpp (e.g. trackNoOdomWheel, resetPositionWithSensor).
static void rotateLocalToField(double forwardIn, double rightIn, double headingRad, double &outX, double &outY) {
    outX = forwardIn * sin(headingRad) + rightIn * cos(headingRad);
    outY = forwardIn * cos(headingRad) - rightIn * sin(headingRad);
}

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

// APRIL TAG POSITION RESET (uses VEX AI Vision Sensor)
// xOffsetIn/yOffsetIn is where the sensor is SUPPOSED to be relative to the
// tag; targetTagAngleDeg (optional) is the expected obj.angle reading (the
// sensor's own rotation relative to the tag, 0-359, per VEX's AI Vision docs)
// at that same pose. This measures where things ACTUALLY are and sets
// x_pos/y_pos to the error between expected and measured -- e.g. if you
// expect (15,15) but the tag shows (14.5,15), x_pos/y_pos becomes (-0.5,0),
// so the rest of the routine (which plans moves relative to that expected
// point) auto-corrects for it.

// Sensor spec constants (verify against the AI Vision Sensor datasheet -- same
// for all 4 sensors, not robot-specific).
const double APRILTAG_SIZE_IN = 6.5;
const double CAMERA_RES_WIDTH_PX = 320.0;
const double CAMERA_HFOV_DEG = 61.0;

// Wraps a degree value to the shortest signed difference, in (-180, 180].
static double wrapAngleDeg(double deg) {
    while (deg > 180) deg -= 360;
    while (deg <= -180) deg += 360;
    return deg;
}

// Does the actual work for whichever AI Vision Sensor is passed in.
// mountForwardIn/mountSideIn: sensor's offset from the robot's center (inches).
// mountHeadingOffsetDeg: which way that sensor points relative to robot-forward
// (0 = front, 90 = right, 180 = back, 270 = left).
// DO NOT CALL THIS DIRECTLY -- use the direction-specific wrappers below.
void resetPositionWithAprilTagSensor(vex::aivision &sensor, double mountForwardIn, double mountSideIn, double mountHeadingOffsetDeg,
                                      int tagId, double xOffsetIn, double yOffsetIn, double targetTagAngleDeg) {
    sensor.tagDetection(true); // must enable before each snapshot; disabled by default
    sensor.takeSnapshot(aivision::ALL_TAGS);

    for (int i = 0; i < sensor.objectCount; i++) {
        if (sensor.objects[i].id != tagId) continue;
        const auto &obj = sensor.objects[i];

        if (!std::isnan(targetTagAngleDeg)) {
            double angleErrorDeg = wrapAngleDeg(obj.angle - targetTagAngleDeg);
            inertial_sensor.setRotation(getInertialHeading() - APRILTAG_ANGLE_SIGN * angleErrorDeg, degrees);
        }

        // Pinhole model: focal length comes from the sensor's known FOV/resolution;
        // apparent tag width gives distance, pixel offset from center gives bearing.
        double focalLengthPx = (CAMERA_RES_WIDTH_PX / 2.0) / tan(degToRad(CAMERA_HFOV_DEG / 2.0));
        double distanceIn = (APRILTAG_SIZE_IN * focalLengthPx) / obj.width;
        double bearingRad = atan2(obj.centerX - CAMERA_RES_WIDTH_PX / 2.0, focalLengthPx);

        // Camera-to-tag displacement in the camera's own forward/right frame.
        double localForwardIn = distanceIn * cos(bearingRad);
        double localRightIn = distanceIn * sin(bearingRad);

        // Rotate into field-aligned axes using the (now heading-corrected) inertial
        // reading plus this sensor's own mount yaw.
        double robotHeadingRad = degToRad(getInertialHeading());
        double camHeadingRad = robotHeadingRad + degToRad(mountHeadingOffsetDeg);
        double tagFromCamX, tagFromCamY;
        rotateLocalToField(localForwardIn, localRightIn, camHeadingRad, tagFromCamX, tagFromCamY);

        // Sensor mount offset, rotated into field axes, used to convert both the
        // measured and expected sensor positions into robot-tracking-center terms.
        double mountX, mountY;
        rotateLocalToField(mountForwardIn, mountSideIn, robotHeadingRad, mountX, mountY);
        double measuredX = -tagFromCamX - mountX;
        double measuredY = -tagFromCamY - mountY;
        double targetX = xOffsetIn - mountX;
        double targetY = yOffsetIn - mountY;

        x_pos = measuredX - targetX;
        y_pos = measuredY - targetY;
        return;
    }
}

void resetPositionWithAprilTagFront(int tagId, double xOffsetIn, double yOffsetIn, double targetTagAngleDeg) {
    resetPositionWithAprilTagSensor(front_ai_vision, front_ai_vision_offsetX, front_ai_vision_offsetY, 0.0, tagId, xOffsetIn, yOffsetIn, targetTagAngleDeg);
}

void resetPositionWithAprilTagRight(int tagId, double xOffsetIn, double yOffsetIn, double targetTagAngleDeg) {
    resetPositionWithAprilTagSensor(right_ai_vision, right_ai_vision_offsetX, right_ai_vision_offsetY, 90.0, tagId, xOffsetIn, yOffsetIn, targetTagAngleDeg);
}

void resetPositionWithAprilTagBack(int tagId, double xOffsetIn, double yOffsetIn, double targetTagAngleDeg) {
    resetPositionWithAprilTagSensor(back_ai_vision, back_ai_vision_offsetX, back_ai_vision_offsetY, 180.0, tagId, xOffsetIn, yOffsetIn, targetTagAngleDeg);
}

void resetPositionWithAprilTagLeft(int tagId, double xOffsetIn, double yOffsetIn, double targetTagAngleDeg) {
    resetPositionWithAprilTagSensor(left_ai_vision, left_ai_vision_offsetX, left_ai_vision_offsetY, 270.0, tagId, xOffsetIn, yOffsetIn, targetTagAngleDeg);
}

