#include <string>
#include <cmath>


// --- Global Variables (snake_case) ---
extern bool is_turning;


extern double x_pos, y_pos;
extern double correct_angle;


// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);


double getInertialHeading(bool normalize = false);
double normalizeTarget(double angle);


void turnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);


void stopChassis(vex::brakeType type = vex::brake);
void resetChassis();
double getLeftRotationDegree();
double getRightRotationDegree();
void correctHeading();
void trackNoOdomWheel();
void trackXYOdomWheel();
void trackXOdomWheel();
void trackYOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);

extern double field_half_size;
void resetPositionWithSensor(vex::distance& sensor, double s_x, double s_y, double sensor_angle_deg, double field_half_size = 72);
void resetPositionFront();
void resetPositionBack();
void resetPositionLeft();
void resetPositionRight();

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
//   this file.
void distanceReset(char xDirection, char yDirection, double expectedXmm, double expectedYmm);

// If tagId is visible to that sensor, sets x_pos/y_pos to the error between
// where the sensor is expected to be (xOffsetIn, yOffsetIn from the tag) and
// where the tag shows it actually is, so the rest of the routine auto-corrects
// for drift. targetTagAngleDeg (optional) is the expected obj.angle reading
// (the AI Vision Sensor's own rotation relative to the tag) at that pose; if
// given, the error between it and the live reading corrects the inertial
// sensor's heading too. Does nothing if the tag isn't visible.
//
// One robot-facing wrapper per mounted AI Vision Sensor -- call whichever
// one matches the direction the tag should be visible from.
void resetPositionWithAprilTagFront(int tagId, double xOffsetIn, double yOffsetIn, double targetTagAngleDeg = NAN);
void resetPositionWithAprilTagBack(int tagId, double xOffsetIn, double yOffsetIn, double targetTagAngleDeg = NAN);
void resetPositionWithAprilTagLeft(int tagId, double xOffsetIn, double yOffsetIn, double targetTagAngleDeg = NAN);
void resetPositionWithAprilTagRight(int tagId, double xOffsetIn, double yOffsetIn, double targetTagAngleDeg = NAN);
