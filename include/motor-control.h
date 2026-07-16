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

// Measures the robot's field position from distance sensors (projecting each
// sensor's raw range reading into field axes using the trusted inertial
// heading) and sets x_pos/y_pos to the ERROR between where you expected the
// robot to be (expectedX, expectedY) and where the sensors actually show it --
// same "expected vs measured" pattern as resetPositionWithAprilTagSensor. E.g.
// if you expect (15,15) but the sensors show (14.5,15), x_pos/y_pos becomes
// (-0.5,0), so the rest of the routine (which plans moves relative to that
// expected point) auto-corrects for it. Does nothing if either sensor can't
// see a wall.
// xDirection: which sensor faces the x-axis wall ('F'=front, 'B'=back, 'R'=right, 'L'=left)
// yDirection: which sensor faces the y-axis wall ('F'=front, 'B'=back, 'R'=right, 'L'=left)
// expectedX / expectedY: the field position you expect the robot to be at
//   right now (measured/CAD'd beforehand for whatever checkpoint you call this at)
void distanceReset(char xDirection, char yDirection, double expectedX, double expectedY);

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
