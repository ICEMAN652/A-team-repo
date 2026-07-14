using namespace vex;


// Format: extern device deviceName;


extern brain Brain;


// VEXcode devices
extern controller controller_1;
extern motor left_chassis1;
extern motor left_chassis2;
extern motor_group left_chassis;
extern motor right_chassis1;
extern motor right_chassis2;
extern motor_group right_chassis;
extern inertial inertial_sensor;
extern digital_out scraper;
extern digital_out descore;
extern digital_out midgoal;
extern rotation horizontal_tracker;
extern rotation vertical_tracker;
extern distance left_sensor;
extern distance back_sensor;
extern distance front_sensor;
extern distance right_sensor;
extern aivision front_ai_vision;
extern aivision back_ai_vision;
extern aivision left_ai_vision;
extern aivision right_ai_vision;






// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
extern double distance_between_wheels;
extern double wheel_distance_in;
extern double distance_kp, distance_ki, distance_kd;
extern double turn_kp, turn_ki, turn_kd;
extern double heading_correction_kp, heading_correction_ki, heading_correction_kd;


extern bool using_horizontal_tracker;
extern bool using_vertical_tracker;
extern double horizontal_tracker_dist_from_center;
extern double vertical_tracker_dist_from_center;
extern double horizontal_tracker_diameter;
extern double vertical_tracker_diameter;
//Distance Reset Setup
extern double left_sensor_offsetX;
extern double left_sensor_offsetY;
extern double right_sensor_offsetX;
extern double right_sensor_offsetY;
extern double front_sensor_offsetX;
extern double front_sensor_offsetY;
extern double back_sensor_offsetX;
extern double back_sensor_offsetY;

// AprilTag Reset Setup -- offset of each AI Vision Sensor from the robot's center
extern double front_ai_vision_offsetX;
extern double front_ai_vision_offsetY;
extern double back_ai_vision_offsetX;
extern double back_ai_vision_offsetY;
extern double left_ai_vision_offsetX;
extern double left_ai_vision_offsetY;
extern double right_ai_vision_offsetX;
extern double right_ai_vision_offsetY;

// +1 if obj.angle increases as the robot turns clockwise, -1 if it decreases.
// Unverified -- rotate the robot a known amount clockwise in front of a tag,
// check which way obj.angle moves, and set this to match before trusting
// the AprilTag heading correction.
extern double APRILTAG_ANGLE_SIGN;



extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
extern double min_output;
extern double max_slew_accel_fwd;
extern double max_slew_decel_fwd;
extern double max_slew_accel_rev;
extern double max_slew_decel_rev;
extern double chase_power;


/**
* Used to initialize code/tasks/devices added using tools in VEXcode Pro.
*
* This should be called at the start of your int main function.
*/
void vexcodeInit(void);

