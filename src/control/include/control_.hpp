#ifndef CONTROL___HPP
#define CONTROL___HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/waypoint.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include "math_.hpp"
#include "geo_.hpp"
#include "drone_controller_.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <livox_ros_driver2/msg/custom_msg.hpp>
// #include <livox_ros_driver2/msg/custom_point.hpp>
#include <string>
#include <iostream>
#include <iomanip>
#include <limits>

#define RATE 10.0

struct PathWaypoint {
    float x;
    float y;
    float z;
    float yaw;
};

struct PathConfig {
    std::vector<PathWaypoint> waypoints;
    float speed;
    float tolerance;
    bool auto_heading;
    bool enable_bspline_smoothing;
    float bspline_smoothness;
    int bspline_degree;
    int num_interpolation_points;
    bool enable_rdp_simplification;
    float rdp_epsilon;
    float slow_down_distance;
    bool enable_lateral_compensation;
    float lateral_compensation_factor;
};

bool payloadDetected(const std::shared_ptr<DroneController>&node);
    /**
     * Check if the payload is detected by verifying if its position is not at the origin (0,0,0).
     * Since PointStamped default constructor initializes position to (0,0,0),
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * 
     * returns:
     * - true if the payload is detected, false otherwise.
     */

bool emberDetected(const std::shared_ptr<DroneController>&node);
    /**
     * Check if the payload is detected by verifying if its position is not at the origin (0,0,0).
     * Since PointStamped default constructor initializes position to (0,0,0),
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * 
     * returns:
     * - true if the payload is detected, false otherwise.
     */    

bool hasReplied(const std::shared_ptr<DroneController>&node, auto& result);
    /**
     * Check if the service call has been replied to.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - result: reference to the service call result. (promise or future)
     * 
     * returns:
     * - true if the service call was successful, false otherwise.
 */

void initFrame(const std::shared_ptr<DroneController>& node, geometry_msgs::msg::PoseStamped &pose);
    /**
     * Initialize the starting pose 'node' with the current position of the drone and reset payload pose.
     * This is used to reset posee or agg_pose (reference to where the last drone position when a control
     * function is called, it will be modified to the last drone position when the function returns.)
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - pose: reference to the geometry_msgs::msg::PoseStamped object to be initialized.
     */
    

void initFrame(const std::shared_ptr<DroneController>& node, geometry_msgs::msg::PoseStamped &pose);
    /**
     * Initialize the starting pose 'node' with the current position of the drone and reset payload pose.
     * This is used to reset posee or agg_pose (reference to where the last drone position when a control
     * function is called, it will be modified to the last drone position when the function returns.)
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - pose: reference to the geometry_msgs::msg::PoseStamped object to be initialized.
     */
    
void executeLocalWaypointMoveVelocity(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float forward_x, float left_y, float up_z, float yaw_angle, float max_velocity, float tolerance);

void setParam(const std::shared_ptr<DroneController>&node, const std::string &id, int integer_value);
    /**
     * Set a parameter on the drone controller node.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - id: string identifier for the parameter to be set. 
     * - integer_value: integer value to set for the parameter.
     */


void setMode(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, const std::string mode);

void takeoff(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt);
    /**
     * Initiate the takeoff procedure for the drone. Set mode to GUIDED, arm the drone, and send the takeoff command.
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - takeoff_alt: float value representing the desired takeoff altitude
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     */

void takeoff_no_confirm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt);
    /**
     * Initiate the takeoff procedure for the drone without having to wait for confirmation. Set mode to GUIDED, arm the drone, and send the takeoff command.
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - takeoff_alt: float value representing the desired takeoff altitude, this is based on the rangefinder reading.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     * - rel_alt: float value representing the relative altitude to be maintained at the correct rangefinder reading.
     */

// void stabilize(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &target_pose, float max_time = 3.0);
    /**
     * Wait for the drone to stably hover at the target pose while constantly updating pose and heading to mitigate drift.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - target_pose: reference to the geometry_msgs::msg::PoseStamped object representing the target pose for stabilization.
     */

void stabilize(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped& posee, float duration = 3.0);


void waitLand(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);
    /**
     * Wait until the drone has landed by checking its altitude.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     */
    

void land(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);
    /**
     * Initiate the landing procedure for the drone by sending a land command.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     */

void safeLanding(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float descent_rate);
    /**
     * Perform controlled safe landing with gradual descent in OFFBOARD mode.
     * Much safer than using LAND mode which can be too fast and cause bouncing.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - posee: reference to current pose (will be updated during descent)
     * - descent_rate: descent speed in m/s (default 0.3 m/s for gentle landing)
     */

void Descend(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee);
    /**
     * Descend the drone to a lower altitude while checking if the payload is detected. Used to take the payload on the ground.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     */
    
void calibrateHoverOrientation(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float max_time, float hover_pitch, float hover_roll, bool calibrate_pitch = true, bool calibrate_roll = true);

void centeringPayload(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float acc, float maxAccel, float x = 1.0, float y = 0.0, float min_center_time = 0.5, float max_center_pitch = 2.5, float max_center_roll = 2.5, float hover_pitch = 0.0, float hover_roll = 0.0, std::string recovery_method = "local_pose", std::string centering_setpoint_mode = "velocity");
    /**
     * Center the payload by adjusting the drone's position based on the payload's current position.
     * The drone will adjust its velocity to center the payload while maintaining a stable hover.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - speed_xy: float value representing the speed at which the drone should move in the XY plane.
     * - acc: tolerance value for determining if the payload is centered.
     * - maxAccel: float value representing the maximum acceleration allowed during centering.
     * - takeAlt: float value representing the altitude to descend to take the payload.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
      * - status: reference to a boolean indicating whether centering is successful or not. Modified by the function (true if successful, false otherwise)
      */

void centering_payload(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, bool &centering_status, float step = 0.2, float tolerance = 10.0, float timeout = 3.0, float max_time = 30.0, float log_interval = 0.5, int min_centered_frames = 1);
    /**
     * Center payload in down-facing camera frame (640x480) using /payload_pose topic data.
     * Topic data is interpreted as x(pixel), y(pixel), and center_dist in z.
     * 
     * parameters:
     * - step: max command velocity per axis (default 0.2 m/s)
     * - tolerance: center distance threshold to mark centered
     * - timeout: stop when no object is detected for this duration (seconds)
     * - max_time: overall maximum centering duration (seconds)
     * - log_interval: status logging interval in seconds (default 0.5)
     * - min_centered_frames: minimum consecutive centered frames before success
     */


void centeringEmber(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float acc, float maxAccel, float min_center_time = 0.25);
    /**
     * Center the payload by adjusting the drone's position based on the payload's current position.
     * The drone will adjust its velocity to center the payload while maintaining a stable hover.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - speed_xy: float value representing the speed at which the drone should move in the XY plane.
     * - acc: tolerance value for determining if the payload is centered.
     * - maxAccel: float value representing the maximum acceleration allowed during centering.
     * - dropAlt: float value representing the altitude to descend to take the payload.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     * - status: reference to a boolean indicating whether centering is successful or not. (true if successful, false otherwise)
     */
    

void centeringGate(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float acc);

// void centeringGate(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee);
//     /**
//      * Center the drone to the gate by adjusting its position based on the current gate pose.
//      * The drone will adjust its velocity to center itself to the gate while maintaining a stable hover.
//      * 
//      * parameters:
//      * - node: shared pointer to the DroneController node instance.
//      * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
//      * 
//      * returns:
//      * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
//      */


void centeringBunder(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float acc, float maxAccel, float min_center_time = 0.25);

// void centeringBunder(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float acc, float maxAccel);

// void centeringBunder(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float offset_x, float offset_y, float acc);
    /**
     * Center the drone to the bunder by adjusting its position based on the current bunder pose.
     * The drone will adjust its velocity to center itself to the bunder while maintaining a stable hover.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - speed_xy: float value representing the speed at which the drone should move in the XY plane.
     * - offset_x: float value representing the x-offset from the bunder center.
     * - offset_y: float value representing the y-offset from the bunder center.
     * - acc: tolerance value for determining if the drone is centered to the bunder.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     * - status: reference to a boolean indicating whether centering is successful or not. (true if successful, false otherwise)
     */
    

void moveToPoint(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float x, float y, float z, float angle, float speed, float tolerance);
    /**
     * Move the drone to a specified point in local coordinates with a given speed and angle.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - x: float value representing the target x-coordinate in local coordinates.
     * - y: float value representing the target y-coordinate in local coordinates.
     * - z: float value representing the target z-coordinate in local coordinates.
     * - angle: float value representing the angle to turn towards in radians.
     * - speed: float value representing the speed at which the drone should move.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */

void moveToPoint(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, geometry_msgs::msg::Pose target, float speed, float tolerance, bool allow_centering_payload = false, bool allow_centering_ember = false, bool disable_z_lock = false);

// void moveToPointLocal(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float x, float y, float z, float speed, bool allow_centering = false);
//     /**
//      * Move the drone to a specified point in local coordinates with a given speed. I (palel) deprecate this function for now since I see no difference with moveToPoint.
//      */
//     geometry_msgs::msg::PoseStamped pose = posee;
//     geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
//     pose.pose.orientation = curr_pose.pose.orientation;
//     float dx = x - posee.pose.position.x;
//     float dy = y - posee.pose.position.y;
//     float distance = sqrt(dx*dx + dy*dy + z*z);
//     float time = abs(distance / speed) * RATE;

//     float tgt_hdg = atan2(dy,dx) - (-(node->getCurrentCompassHdg().data * 3.14/180) + 1.5708); 
//     // moveToPoint(node, rate, local_pos_pub, posee, 0.0, 0, 0, tgt_hdg, 0.3);
//     moveToPoint(node, rate, posee, 0.0, 0, 0, tgt_hdg, 1.2);
//     pose.pose.orientation = posee.pose.orientation;
    
//     for (int i = 0; i < time && rclcpp::ok(); i++) {
//         float progress = (float)i/time;
//         pose.pose.position.x = dx * progress + posee.pose.position.x;
//         pose.pose.position.y = dy * progress + posee.pose.position.y;
//         pose.pose.position.z = z * progress + posee.pose.position.z;
//         local_pos_pub->publish(pose);
        
//         rclcpp::spin_some(node);
//         rate.sleep();
//         curr_pose = node->getCurrentLocalPose();
//         RCLCPP_INFO(node->get_logger(), "MOVE-- x: %f | y: %f | z: %f", curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z);

//         if((node->getCurrentPosePayload().pose.position.x != 0.000 || node->getCurrentPosePayload().pose.position.y != 0.000) && allow_centering){
//             // distancee_to_payload = sqrt((pose.pose.position.x - posee.pose.position.x) * (pose.pose.position.x - posee.pose.position.x) + (pose.pose.position.y - posee.pose.position.y) * (pose.pose.position.y - posee.pose.position.y));
//             break;
//         }
//     }

//     posee = pose;
// }

void landingFaux(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee);

void landingFauxPose(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee);
    /**
     * Descend the drone to a lower altitude using a faux pose method.
     * This method is used to land the drone by gradually decreasing its altitude.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */
    

void holdPosition(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped& posee, float duration);
    /**
     * Wait for a specified duration. In guided mode, the drone should hold the current position when doing nothing.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - duration: float value representing the duration in seconds to hold the position.
     */
    

void waitForPosition(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, mavros_msgs::msg::GlobalPositionTarget raw, double tolerance = 0.00001, bool allow_centering = false, float center_dist = 5.0);
    /**
     * Wait until the drone reaches a specified global position by checking the distance to the target position. Used when the drone moves using waypoint in AUTO mode.
     * (Although its probably better to listen to mavlink's current_wp). see: https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - raw: mavros_msgs::msg::GlobalPositionTarget object containing the target global position.
     * - tolerance: double value representing the tolerance for reaching the target position in degrees.
     * - allow_centering: boolean value indicating whether to allow centering the drone based on the payload position.
     * - center_dist: float value representing the distance in meters to center the drone based on the payload position. (Not used for now)
     */
    

void sendGlobalRaw(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float latt, float lonn, float altt, bool allow_centering = false, float center_dist = 5.0, float tolerance = 0.0000025);
    /**
     * Send a raw global position target to the drone and wait until the drone reaches the target position.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - latt: float value representing the target latitude in degrees.
     * - longg: float value representing the target longitude in degrees.
     * - altt: float value representing the target altitude in meters.
     * - allow_centering: boolean value indicating whether to allow centering the drone based on the payload position.
     * - center_dist: float value representing the distance in meters to center the drone based on the payload position.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */

void sendGlobalRawAsync(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float latt, float lonn, float altt, bool allow_centering, float center_dist);

void sendGlobalRawFromXYZ(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, LatLonAlt zero, double zero_hdg, float x, float y, float rel_alt, bool allow_centerin = false, float center_dist = 5.0);

void correctAltitude(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, double height, float& corrected_rel_alt, double max_speed = 0.25);
    /**
     * Correct the drone's altitude to a specified rangefinder reading by adjusting its velocity.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - vel_pub: shared pointer to the rclcpp::Publisher for publishing velocity commands.
     * - height: double value representing the target altitude in meters.
     * 
     * returns:
     * - corrected_rel_alt: The correct relative altitude after adjusting the drone's position. ready to be used for sendGlobalRaw.
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */

void controlServo(const std::shared_ptr<DroneController>&node, int channel, int pwm);

void controlServoRepeated(const std::shared_ptr<DroneController>& node, int channel, int pwm, int repeat = 3);

void pushMission(const std::shared_ptr<DroneController>&node, std::vector<mavros_msgs::msg::Waypoint> waypoints);

void clearMission(const std::shared_ptr<DroneController>&node);

void waitForWP(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, int seq);

void arm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);

void disarm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);

void moveWithLaser(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float x, float y, float z, float max_speed, float tolerance, bool allow_centering_payload = false, bool allow_centering_ember = false, float koreksi_max_speed = 1.0, float timeout = -1.0);

void holdAtLidarRanges(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float front, float left, float back, float right, bool allow_centering_payload = false, bool allow_centering_ember = false);

float getYawError(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate);

void printTitik(std::vector<double>& lat_indoor, std::vector<double>& lon_indoor, std::vector<double>& lat_outdoor, std::vector<double>& lon_outdlat_outdoor, double lat_takeoff, double lon_takeoff);

void executeLocalWaypointMove(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float forward_x, float left_y, float up_z, float yaw_angle, float tolerance);
    /**
     * Execute a local waypoint movement by publishing position commands to /mavros/local_position/pose
     * Movement is relative to the current drone position in the local frame
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final position
     * - forward_x: distance to move forward (positive) or backward (negative) in meters
     * - left_y: distance to move left (positive) or right (negative) in meters
     * - up_z: distance to move up (positive) or down (negative) in meters
     * - yaw_angle: target yaw angle in radians (0 = no change, use current heading)
     * - tolerance: distance tolerance to consider waypoint reached in meters
     */

void executeLocalWaypointRotation(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float rotation_angle);
    /**
     * Execute a rotation in place by publishing position commands with updated orientation
     * Rotation is relative to current heading
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final orientation
     * - rotation_angle: angle to rotate in RADIANS (positive = right/clockwise, negative = left/counter-clockwise)
     */

void rotateByDegrees(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float degrees);
    /**
     * Execute a rotation in place - EASIER VERSION with degrees instead of radians
     * Rotation is relative to current heading
     * 
     * USAGE EXAMPLES:
     * - rotateByDegrees(node, rate, posee, 90.0);   // Rotate 90° RIGHT (clockwise)
     * - rotateByDegrees(node, rate, posee, -90.0);  // Rotate 90° LEFT (counter-clockwise)
     * - rotateByDegrees(node, rate, posee, 180.0);  // Turn around (U-turn)
     * - rotateByDegrees(node, rate, posee, 45.0);   // Rotate 45° RIGHT
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final orientation
     * - degrees: angle to rotate in DEGREES (positive = RIGHT/clockwise, negative = LEFT/counter-clockwise)
     */

void strafeRight(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float distance, float tolerance);
    /**
     * Strafe (move sideways) to the right relative to drone's current heading
     * This is a convenience wrapper for executeLocalWaypointMove with left_y = -distance
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final position
     * - distance: distance to strafe right in meters (positive value)
     * - tolerance: distance tolerance to consider waypoint reached in meters
     */

void strafeLeft(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float distance, float tolerance);
    /**
     * Strafe (move sideways) to the left relative to drone's current heading
     * This is a convenience wrapper for executeLocalWaypointMove with left_y = +distance
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final position
     * - distance: distance to strafe left in meters (positive value)
     * - tolerance: distance tolerance to consider waypoint reached in meters
     */

bool detectGatePosts(const std::shared_ptr<DroneController>&node, float &left_post_angle, float &right_post_angle, float &left_post_distance, float &right_post_distance, float gate_width = 1.5, float max_detection_range = 10.0);
    /**
     * Detect two gate posts using 2D lidar scan
     * Gate posts are identified as two distinct obstacles with approximately gate_width distance apart
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - left_post_angle: output angle of left post in radians (relative to drone heading)
     * - right_post_angle: output angle of right post in radians (relative to drone heading)
     * - left_post_distance: output distance to left post in meters
     * - right_post_distance: output distance to right post in meters
     * - gate_width: expected width of the gate in meters (default 1.5m)
     * - max_detection_range: maximum range to consider for detection (default 10.0m)
     * 
     * returns:
     * - true if gate posts are successfully detected, false otherwise
     */

void centeringGateLidar(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float gate_width, float tolerance, bool &status, float max_time = 30.0);
    /**
     * Center the drone between two gate posts using 2D lidar detection (POSITION CONTROL)
     * The drone will strafe left/right to align itself with the center of the gate
     * Uses position setpoints with step-wise movement (30cm per iteration)
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final position
     * - gate_width: expected width of the gate in meters (e.g., 1.5m)
     * - tolerance: centering tolerance in meters (how close to center is acceptable)
     * - status: output status indicating success (true) or failure (false)
     * - max_time: maximum time to attempt centering in seconds (default 30.0)
     */

void centeringGateLidarVelocity(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float gate_width, float tolerance, bool &status, float max_velocity = 0.2, float proportional_gain = 0.5, float max_time = 30.0);
    /**
     * Center the drone between two gate posts using 2D lidar detection (VELOCITY CONTROL)
     * The drone will smoothly move left/right using proportional velocity control
     * More smooth and safer than position control version
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final position
     * - gate_width: expected width of the gate in meters (e.g., 1.5m)
     * - tolerance: centering tolerance in meters (how close to center is acceptable)
     * - status: output status indicating success (true) or failure (false)
     * - max_velocity: maximum lateral velocity in m/s (default 0.2 m/s for safety)
     * - proportional_gain: Kp gain for proportional control (default 0.5)
     * - max_time: maximum time to attempt centering in seconds (default 30.0)
     */

bool detectGatePostsNear(const std::shared_ptr<DroneController>&node, float &left_post_angle, float &right_post_angle, float &left_post_distance, float &right_post_distance, float gate_width = 1.5, float max_detection_range = 10.0, float forward_zone_min = 0.5, float max_lateral_angle = 0.785);
    /**
     * Detect NEAREST FORWARD gate posts using 2D lidar scan - ENHANCED for multiple gates
     * Prioritizes gate directly in front when multiple gates are detected (side-by-side scenario)
     * 
     * NEW STRATEGY:
     * 1. Filters posts by forward zone (x > forward_zone_min) to ignore side/back obstacles
     * 2. Finds all gate candidates matching gate_width tolerance
     * 3. Scores each gate by forward alignment (angle to center point)
     * 4. Selects gate with BEST alignment (closest to 0° heading)
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - left_post_angle: output angle of left post in radians (relative to drone heading)
     * - right_post_angle: output angle of right post in radians (relative to drone heading)
     * - left_post_distance: output distance to left post in meters
     * - right_post_distance: output distance to right post in meters
     * - gate_width: expected width of the gate in meters (default 1.5m)
     * - max_detection_range: maximum range to consider for detection (default 10.0m)
     * - forward_zone_min: minimum forward distance to filter posts (default 0.5m)
     * - max_lateral_angle: maximum angle to gate center in radians (default 0.785 = 45°)
     * 
     * returns:
     * - true if nearest forward gate is detected, false otherwise
     */

void centeringGateNear(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float gate_width, float tolerance, bool &status, float max_velocity = 0.2, float proportional_gain = 0.5, float max_time = 30.0, float forward_zone_min = 0.5, float max_lateral_angle = 0.785);
    /**
     * Center the drone to NEAREST FORWARD GATE using velocity control - ENHANCED for multiple gates
     * Uses detectGatePostsNear() to prioritize gate directly in front
     * Perfect for scenarios with multiple side-by-side gates
     * 
     * STRATEGY:
     * - Filters gates by forward zone and lateral angle
     * - Always selects gate with best forward alignment (closest to 0° heading)
     * - Smooth proportional velocity control for centering
     * - Prevents confusion when multiple gates are present
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final position
     * - gate_width: expected width of the gate in meters (e.g., 1.5m)
     * - tolerance: centering tolerance in meters (how close to center is acceptable)
     * - status: output status indicating success (true) or failure (false)
     * - max_velocity: maximum lateral velocity in m/s (default 0.2 m/s)
     * - proportional_gain: Kp gain for proportional control (default 0.5)
     * - max_time: maximum time to attempt centering in seconds (default 30.0)
     * - forward_zone_min: minimum forward distance to consider (default 0.5m)
     * - max_lateral_angle: maximum angle to gate center in radians (default 0.785 = 45°)
     */

void centeringGateLidar3D(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float gate_width, float lateral_tolerance, float vertical_tolerance, bool &status, float max_velocity = 0.2, float proportional_gain = 0.5, float max_time = 30.0, int min_points_vertical = 10, int min_points_horizontal = 30);
    /**
     * Center the drone to gate using 3D LIDAR (Velodyne VLP-16) with simple clustering
     * Detects left and right gate poles only (ignores top/bottom)
     * 
     * STRATEGY:
     * - Process PointCloud2 from /scan topic (Velodyne VLP-16)
     * - Simple spatial clustering to find vertical poles (left & right gate posts)
     * - Calculate lateral offset from gate center
     * - Smooth proportional velocity control for centering
     * - Ignores top and bottom of gate (focuses on vertical poles only)
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose that will be updated with final position
     * - gate_width: expected width between left and right poles in meters (e.g., 1.5m)
     * - lateral_tolerance: lateral centering tolerance in meters (default 0.10m)
     * - vertical_tolerance: vertical centering tolerance in meters (default 0.10m)
     * - status: output status indicating success (true) or failure (false)
     * - max_velocity: maximum velocity in m/s (default 0.2 m/s)
     * - proportional_gain: Kp gain for proportional control (default 0.5)
     * - max_time: maximum time to attempt centering in seconds (default 30.0)
     * - min_points_vertical: minimum vertical points to consider as pole (default 10)
     * - min_points_horizontal: minimum horizontal points spread to consider as pole (default 30)
     * 
     * returns:
     * - status: true if centering successful, false if timeout or error
     */

void centeringGateLivoxSimple(
    const std::shared_ptr<DroneController>&node,
    rclcpp::Rate &rate,
    geometry_msgs::msg::PoseStamped &posee,
    float gate_width,
    float tolerance,
    bool &status,
    float max_velocity = 0.2,
    float proportional_gain = 0.5,
    float max_time = 30.0,
    bool test_mode = false);
    /**
     * SIMPLE gate centering with Livox Mid-360 using grid binning (similar to 2D lidar)
     * 
     * ULTRA-SIMPLE STRATEGY:
     * 1. Filter point cloud by ROI (forward 1.5-2.5m, lateral ±2m, height 0-2m)
     * 2. Grid binning: divide lateral range into bins, count points per bin
     * 3. Find 2 peak bins (highest point counts) = left & right poles
     * 4. Validate gate width (1.0-2.0m)
     * 5. Calculate lateral offset and apply proportional control
     * 6. No complex clustering, no nested loops - just bin counting!
     * 
     * FEATURES:
     * - O(n) complexity - very fast
     * - Easy to maintain - straightforward grid logic
     * - Similar to 2D lidar centering approach
     * - Lateral centering only (no vertical control)
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - posee: reference to current pose (not used, but kept for consistency)
     * - gate_width: expected gate width in meters (default 1.5m)
     * - tolerance: lateral centering tolerance in meters (default 0.15m)
     * - status: output status indicating success (true) or failure (false)
     * - max_velocity: maximum velocity in m/s (default 0.2 m/s)
     * - proportional_gain: Kp gain for lateral control (default 0.5)
     * - max_time: maximum time to attempt centering in seconds (default 30.0)
     * 
     * returns:
     * - status: true if centering successful, false if timeout or error
     */

void centeringGateLivoxSimpleRear(
    const std::shared_ptr<DroneController>&node,
    rclcpp::Rate &rate,
    geometry_msgs::msg::PoseStamped &posee,
    float gate_width,
    float tolerance,
    bool &status,
    float max_velocity = 0.2,
    float proportional_gain = 0.5,
    float max_time = 30.0,
    bool test_mode = false);

void centeringGateLivoxFast(
    const std::shared_ptr<DroneController>&node,
    rclcpp::Rate &rate,
    geometry_msgs::msg::PoseStamped &posee,
    float gate_width,
    float tolerance,
    bool &status,
    float max_velocity = 0.2,
    float proportional_gain = 0.5,
    float max_time = 30.0,
    bool test_mode = false);

void followPath(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, const std::vector<PathWaypoint>& waypoints, float speed, float tolerance, bool allow_centering_payload = false, bool allow_centering_ember = false, bool disable_z_lock = false, bool stabilize_at_waypoint = true, float stabilize_duration = 2.0);

void followPathFromConfig(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, const PathConfig& config);

std::vector<PathWaypoint> smoothPathBSpline(const std::vector<PathWaypoint>& waypoints, float smoothness, int degree, int num_points);

std::vector<PathWaypoint> ramerDouglasPeucker(const std::vector<PathWaypoint>& path, float epsilon);

void followPathVelocity(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, const std::vector<PathWaypoint>& path, float max_speed, float step_tolerance, float slow_down_distance, bool enable_lateral_comp, float lateral_comp_factor);
    
#endif  // CONTROL__CONTROL__HPP_
