#include "control_.hpp"
#include "math_.hpp"
#include "geo_.hpp"
#include "drone_controller_.hpp"
#include <iostream>
#include <vector>
#include <string>

#define RATE 10.0

float takeoff_altitude;
float forward_distance;
float gate_width;
float centering_tolerance;
float waypoint_tolerance;
std::string control_mode;  
float max_velocity;
float proportional_gain;
float rotation_angle; 
float rotation_radians;
float waypoint_speed;

float lateral_tolerance;
float vertical_tolerance;
int min_points_vertical;
int min_points_horizontal;
float max_time = 20.0;  // Default timeout for centering 

float turn_radius;
float turn_speed;
int num_waypoints;
float waypoint_tolerance_bezier;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate rate(RATE);
    auto node = std::make_shared<DroneController>();
    
    node->declare_parameter<float>("gate_centering.takeoff_altitude", 2.0);
    node->declare_parameter<float>("gate_centering.forward_distance", 3.0);
    node->declare_parameter<float>("gate_centering.gate_width", 1.5);
    node->declare_parameter<float>("gate_centering.centering_tolerance", 0.15);
    node->declare_parameter<float>("gate_centering.waypoint_tolerance", 0.3);
    node->declare_parameter<std::string>("gate_centering.control_mode", "velocity");
    node->declare_parameter<float>("gate_centering.max_velocity", 0.2);
    node->declare_parameter<float>("gate_centering.proportional_gain", 0.5);
    node->declare_parameter<float>("gate_centering.rotation_angle", 90.0);
    node->declare_parameter<float>("gate_centering.waypoint_speed", 0.3);
    
    // Clustering parameters
    node->declare_parameter<float>("gate_centering.detection_range_min", 1.5);
    node->declare_parameter<float>("gate_centering.detection_range_max", 2.5);
    node->declare_parameter<float>("gate_centering.roi_lateral", 2.0);
    node->declare_parameter<float>("gate_centering.cluster_epsilon", 0.15);
    node->declare_parameter<int>("gate_centering.min_cluster_points", 10);
    node->declare_parameter<float>("gate_centering.min_pole_height", 0.5);
    node->declare_parameter<float>("gate_centering.target_height", 0.75);

    // turn param
    // node->declare_parameter<float>("bezier_turn.turn_radius", 2.5);
    // node->declare_parameter<float>("bezier_turn.turn_speed", 0.3);
    // node->declare_parameter<int>("bezier_turn.num_waypoints", 8);
    // node->declare_parameter<float>("bezier_turn.waypoint_tolerance", 0.2);
    // node->get_parameter("bezier_turn.turn_radius", turn_radius);
    // node->get_parameter("bezier_turn.turn_speed", turn_speed);
    // node->get_parameter("bezier_turn.num_waypoints", num_waypoints);
    // node->get_parameter("bezier_turn.waypoint_tolerance", waypoint_tolerance_bezier);

    node->get_parameter("gate_centering.takeoff_altitude", takeoff_altitude);
    node->get_parameter("gate_centering.forward_distance", forward_distance);
    node->get_parameter("gate_centering.gate_width", gate_width);
    node->get_parameter("gate_centering.centering_tolerance", centering_tolerance);
    node->get_parameter("gate_centering.waypoint_tolerance", waypoint_tolerance);
    node->get_parameter("gate_centering.control_mode", control_mode);
    node->get_parameter("gate_centering.max_velocity", max_velocity);
    node->get_parameter("gate_centering.proportional_gain", proportional_gain);
    node->get_parameter("gate_centering.rotation_angle", rotation_angle);
    node->get_parameter("gate_centering.waypoint_speed", waypoint_speed);
    rotation_radians = rotation_angle * M_PI / 180.0;
    
    RCLCPP_INFO(node->get_logger(), "=== Centering Basic ===");
    RCLCPP_INFO(node->get_logger(), "Takeoff Alt: %.2f m", takeoff_altitude);
    RCLCPP_INFO(node->get_logger(), "Forward Dist: %.2f m", forward_distance);
    RCLCPP_INFO(node->get_logger(), "Lebar Gate: %.2f m", gate_width);
    RCLCPP_INFO(node->get_logger(), "Centering Tolerance: %.2f m", centering_tolerance);
    RCLCPP_INFO(node->get_logger(), "Control Mode: %s", control_mode.c_str());
    if (control_mode == "velocity") {
        RCLCPP_INFO(node->get_logger(), "Max Velocity: %.2f m/s", max_velocity);
        RCLCPP_INFO(node->get_logger(), "Proportional Gain: %.2f", proportional_gain);
    }
    
    geometry_msgs::msg::PoseStamped posee;
    initFrame(node, posee);
    
    while(rclcpp::ok() && node->getCurrentLocalPose().pose.position.z == 0.0) {
        RCLCPP_INFO(node->get_logger(), "Wait local pose data...");
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Taking off %.2f meters...", takeoff_altitude);
    takeoff(node, rate, posee, takeoff_altitude);
    holdPosition(node, rate, posee, 10.0);

    RCLCPP_INFO(node->get_logger(), "MOVING FORWARD %.2f METERS...", forward_distance);
    executeLocalWaypointMoveVelocity(node, rate, posee, 10, 0.0, 0.0, 0.0, 2.0, waypoint_tolerance);
    holdPosition(node, rate, posee, 2.0);

    // RCLCPP_ERROR(node->get_logger(), "LANDINGG...");
    // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    // holdPosition(node, rate, posee, 2.0);

    // RCLCPP_WARN(node->get_logger(), "set mode land...");
    // setMode(node, rate, "land");
    // waitLand(node, rate);

    // bool centering_status = false;

    // // RCLCPP_INFO(node->get_logger(), "moving forward %.2f meters again...", forward_distance);
    // // executeLocalWaypointMove(node, rate, posee, forward_distance, 0.0, 0.0, 0.0, waypoint_tolerance);
    // // holdPosition(node, rate, posee, 1.0);

    // // centeringGateLivoxFast(node, rate, posee, gate_width, centering_tolerance, centering_status, waypoint_speed, proportional_gain, max_time);
    // centeringGateLivoxSimple(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, max_time);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESS!");
    //     RCLCPP_INFO(node->get_logger(), "============= MAJUUU.... ===============");
    //     RCLCPP_INFO(node->get_logger(), "MOVING FORWARD %.2f METERS...", forward_distance);
    //     executeLocalWaypointMoveVelocity(node, rate, posee, 10.0, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 0.5);
    //     // executeLocalWaypointMoveVelocity(node, rate, posee, forward_distance, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
	// // safeLanding(node, rate, posee, 0.3);
    //     // executeLocalWaypointMove(node, rate, posee, forward_distance + 9.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     // holdPosition(node, rate, posee, 2.0);
        
    //     // RCLCPP_INFO(node->get_logger(), "Mission complete! Performing safe landing...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    // } else {
    //     // RCLCPP_ERROR(node->get_logger(), "Gate centering FAILED! Returning and landing safely...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    //     RCLCPP_WARN(node->get_logger(), "GAGAL CENTERING LANDING....!!!");
    //     setMode(node, rate, "AUTO.LAND");
    //     waitLand(node, rate);
    // }

    // RCLCPP_INFO(node->get_logger(), "Step 3: Strafing right 2.0 meters...");
    // strafeRight(node, rate, posee, 1.2, waypoint_tolerance);
    // holdPosition(node, rate, posee, 0.5);

    // centeringGateLivoxSimple(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, max_time);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESS!");
    //     RCLCPP_INFO(node->get_logger(), "============= MAJUUU.... ===============");
    //     RCLCPP_INFO(node->get_logger(), "MOVING FORWARD %.2f METERS...", forward_distance);
    //     executeLocalWaypointMoveVelocity(node, rate, posee, 7.0, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 0.5);
    //     // executeLocalWaypointMoveVelocity(node, rate, posee, forward_distance, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
	// // safeLanding(node, rate, posee, 0.3);
    //     // executeLocalWaypointMove(node, rate, posee, forward_distance + 9.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     // holdPosition(node, rate, posee, 2.0);
        
    //     // RCLCPP_INFO(node->get_logger(), "Mission complete! Performing safe landing...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    // } else {
    //     // RCLCPP_ERROR(node->get_logger(), "Gate centering FAILED! Returning and landing safely...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    //     RCLCPP_WARN(node->get_logger(), "GAGAL CENTERING LANDING....!!!");
    //     setMode(node, rate, "AUTO.LAND");
    //     waitLand(node, rate);
    // }

    // RCLCPP_INFO(node->get_logger(), "Step 3: Strafing right 2.0 meters...");
    // strafeRight(node, rate, posee, 7.0, waypoint_tolerance);
    // holdPosition(node, rate, posee, 0.5);

    // RCLCPP_INFO(node->get_logger(), "=============== MUNDURRR REARRR ================");
    // // executeLocalWaypointMove(node, rate, posee, -4.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    // executeLocalWaypointMoveVelocity(node, rate, posee, -2.0, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    // holdPosition(node, rate, posee, 0.5);

    // centeringGateLivoxSimpleRear(node, rate, posee, gate_width, centering_tolerance, centering_status, waypoint_speed, proportional_gain, max_time);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
    //     RCLCPP_INFO(node->get_logger(), "=============== MUNDURRR REARRR ================");
    //     // executeLocalWaypointMove(node, rate, posee, -4.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     executeLocalWaypointMoveVelocity(node, rate, posee, -16.5, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 0.5);
    // } else {
    //     RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
    //     setMode(node, rate, "AUTO.LAND");
    //     waitLand(node, rate);
    //     rclcpp::shutdown();
    //     return 0;
    // }

    // RCLCPP_INFO(node->get_logger(), "ROTATEEE.... %.2f ", rotation_angle);
    // rotateByDegrees(node, rate, posee, -30.0);
    // holdPosition(node, rate, posee, 0.5);

    // RCLCPP_INFO(node->get_logger(), "Step 3: Strafing right 2.0 meters...");
    // strafeRight(node, rate, posee, 0.3, waypoint_tolerance);
    // holdPosition(node, rate, posee, 0.5);

    // RCLCPP_INFO(node->get_logger(), "MAJUUU......");
    // executeLocalWaypointMoveVelocity(node, rate, posee, -2.7, 0.0, 0.0, 0.0, 1.0, waypoint_tolerance);
    // holdPosition(node, rate, posee, 0.5);

    // RCLCPP_INFO(node->get_logger(), "ROTATEEE.... %.2f ", rotation_angle);
    // rotateByDegrees(node, rate, posee, 20.0);
    // holdPosition(node, rate, posee, 0.5);

    // centeringGateLivoxSimpleRear(node, rate, posee, gate_width, centering_tolerance, centering_status, waypoint_speed, proportional_gain, max_time);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
    //     RCLCPP_INFO(node->get_logger(), "=============== MUNDURRR REARRR ================");
    //     // executeLocalWaypointMove(node, rate, posee, -4.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     executeLocalWaypointMoveVelocity(node, rate, posee, -1.0, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 1.0);
    // } else {
    //     RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
    //     // setMode(node, rate, "AUTO.LAND");
    //     // waitLand(node, rate);
    //     rclcpp::shutdown();
    //     return 0;
    // }

    // RCLCPP_INFO(node->get_logger(), "Step 3: Strafing right 2.0 meters...");
    // strafeLeft(node, rate, posee, 7.0, waypoint_tolerance);
    // holdPosition(node, rate, posee, 0.5);

    // centeringGateLivoxSimple(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, max_time);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESS!");
    //     RCLCPP_INFO(node->get_logger(), "============= MAJUUU.... ===============");
    //     RCLCPP_INFO(node->get_logger(), "MOVING FORWARD %.2f METERS...", forward_distance);
    //     executeLocalWaypointMoveVelocity(node, rate, posee, 4.0, 0.0, 0.0, 0.0, 1.0, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 0.5);
    //     RCLCPP_WARN(node->get_logger(), "GOOD MASUKKKK.... LANDINGG....!!!");
    //     setMode(node, rate, "AUTO.LAND");
    //     waitLand(node, rate);
    //     // executeLocalWaypointMoveVelocity(node, rate, posee, forward_distance, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
	// // safeLanding(node, rate, posee, 0.3);
    //     // executeLocalWaypointMove(node, rate, posee, forward_distance + 9.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     // holdPosition(node, rate, posee, 2.0);
        
    //     // RCLCPP_INFO(node->get_logger(), "Mission complete! Performing safe landing...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    // } else {
    //     // RCLCPP_ERROR(node->get_logger(), "Gate centering FAILED! Returning and landing safely...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    //     RCLCPP_WARN(node->get_logger(), "GAGAL CENTERING LANDING....!!!");
    //     setMode(node, rate, "AUTO.LAND");
    //     waitLand(node, rate);
    // }
    // =====================================================================================

    // RCLCPP_INFO(node->get_logger(), "Step 4: Rotating %.2f degrees to the right...", rotation_angle);
    // // executeLocalWaypointRotation(node, rate, posee, rotation_radians);
    // rotateByDegrees(node, rate, posee, -15.0);
    // holdPosition(node, rate, posee, 1.0);

    // centeringGateLivoxSimple(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, max_time);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESS!");
    //     RCLCPP_INFO(node->get_logger(), "============= MAJUUU.... ===============");
    //     RCLCPP_INFO(node->get_logger(), "MOVING FORWARD %.2f METERS...", forward_distance);
    //     executeLocalWaypointMoveVelocity(node, rate, posee, 6.0, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 2.0);
    //     // executeLocalWaypointMoveVelocity(node, rate, posee, forward_distance, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
	// // safeLanding(node, rate, posee, 0.3);
    //     // executeLocalWaypointMove(node, rate, posee, forward_distance + 9.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     // holdPosition(node, rate, posee, 2.0);
        
    //     // RCLCPP_INFO(node->get_logger(), "Mission complete! Performing safe landing...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    // } else {
    //     // RCLCPP_ERROR(node->get_logger(), "Gate centering FAILED! Returning and landing safely...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    //     RCLCPP_WARN(node->get_logger(), "GAGAL CENTERING LANDING....!!!");
    //     setMode(node, rate, "AUTO.LAND");
    //     waitLand(node, rate);
    // }

    // RCLCPP_INFO(node->get_logger(), "Step 3: Strafing right 2.0 meters...");
    // strafeLeft(node, rate, posee, 0.3, waypoint_tolerance);
    // holdPosition(node, rate, posee, 1.0);

    // RCLCPP_INFO(node->get_logger(), "CENTERINGG REARRR...");
    // centeringGateLivoxSimpleRear(node, rate, posee, gate_width, centering_tolerance, centering_status, waypoint_speed, proportional_gain, max_time);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
    //     RCLCPP_INFO(node->get_logger(), "=============== MUNDURRR REARRR ================");
    //     // executeLocalWaypointMove(node, rate, posee, -4.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     executeLocalWaypointMoveVelocity(node, rate, posee, -5.0, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 1.0);
    // } else {
    //     RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
    //     setMode(node, rate, "AUTO.LAND");
    //     waitLand(node, rate);
    //     rclcpp::shutdown();
    //     return 0;
    // }

    // RCLCPP_INFO(node->get_logger(), "Step 4: Rotating %.2f degrees to the right...", rotation_angle);
    // // executeLocalWaypointRotation(node, rate, posee, rotation_radians);
    // rotateByDegrees(node, rate, posee, 15.0);
    // holdPosition(node, rate, posee, 1.0);

    // RCLCPP_INFO(node->get_logger(), "MAJU DIKIT...");
    // executeLocalWaypointMoveVelocity(node, rate, posee, -0.5, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    // holdPosition(node, rate, posee, 1.0);

    // RCLCPP_INFO(node->get_logger(), "GESER KIRI...");
    // strafeLeft(node, rate, posee, 0.3, waypoint_tolerance);
    // holdPosition(node, rate, posee, 1.0);

    // RCLCPP_INFO(node->get_logger(), "CENTERINGG REARRR...");
    // centeringGateLivoxSimpleRear(node, rate, posee, gate_width, centering_tolerance, centering_status, waypoint_speed, proportional_gain, max_time);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
    //     RCLCPP_INFO(node->get_logger(), "=============== MUNDURRR REARRR ================");
    //     // executeLocalWaypointMove(node, rate, posee, -4.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     executeLocalWaypointMoveVelocity(node, rate, posee, -5.0, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 1.0);
    // } else {
    //     RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
    //     setMode(node, rate, "AUTO.LAND");
    //     waitLand(node, rate);
    //     rclcpp::shutdown();
    //     return 0;
    // }

    // // RCLCPP_INFO(node->get_logger(), "ROTATING right...", rotation_angle);
    // // executeLocalWaypointRotation(node, rate, posee, -rotation_radians);
    // // holdPosition(node, rate, posee, 1.0);
    // RCLCPP_INFO(node->get_logger(), "MOVING FORWARD %.2f METERS...", forward_distance);
    // // executeLocalWaypointMoveVelocity(node, rate, posee, 2.0, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    // executeLocalWaypointMoveVelocity(node, rate, posee, forward_distance, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    // holdPosition(node, rate, posee, 1.0);

    // =====================================================================================
    // RCLCPP_INFO(node->get_logger(), "right turn %.2f degrees...", rotation_angle);
    // executeBezierTurnRight(node, rate, posee, 90.0);
    // holdPosition(node, rate, posee, 1.0);
    // =====================================================================================


    // holdPosition(node, rate, posee, 1.0);

    // RCLCPP_INFO(node->get_logger(), "GESER KANAN....");
    // strafeRight(node, rate, posee, 0.5, 1.0);
    // holdPosition(node, rate, posee, 1.0);

    // // RCLCPP_INFO(node->get_logger(), "ROTATING right...", rotation_angle);
    // // executeLocalWaypointRotation(node, rate, posee, -20);
    // // holdPosition(node, rate, posee, 1.0);

    // centeringGateLivoxSimple(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, max_time);
    // if (centering_status) {
    //     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESS!");
    //     RCLCPP_INFO(node->get_logger(), "============= LANDING.... ===============");
    //     RCLCPP_INFO(node->get_logger(), "MOVING FORWARD %.2f METERS...", forward_distance);
    //     executeLocalWaypointMoveVelocity(node, rate, posee, 5.5, 0.0, 0.0, 0.0, max_velocity, waypoint_tolerance);
    //     holdPosition(node, rate, posee, 2.0);
	// // safeLanding(node, rate, posee, 0.3);
    //     // executeLocalWaypointMove(node, rate, posee, forward_distance + 9.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    //     // holdPosition(node, rate, posee, 2.0);
        
    //     // RCLCPP_INFO(node->get_logger(), "Mission complete! Performing safe landing...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    // } else {
    //     // RCLCPP_ERROR(node->get_logger(), "Gate centering FAILED! Returning and landing safely...");
    //     // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    //     RCLCPP_WARN(node->get_logger(), "GAGAL CENTERING LANDING....!!!");
    //     setMode(node, rate, "AUTO.LAND");
    //     waitLand(node, rate);
    // }

    // rotate right
    // RCLCPP_INFO(node->get_logger(), "ROTATING right...", rotation_angle);
    // executeLocalWaypointRotation(node, rate, posee, -rotation_angle);

    // executeLocalWaypointMoveVelocity(node, rate, posee, 8.0, 0.0, 0.0, 0.0, waypoint_speed, waypoint_tolerance);
    // holdPosition(node, rate, posee, 1.0);
    
    // RCLCPP_ERROR(node->get_logger(), "LANDINGG...");
    // safeLanding(node, rate, posee, 0.3); // 0.3 m/s descent rate - gentle and safe
    // holdPosition(node, rate, posee, 2.0);
    RCLCPP_WARN(node->get_logger(), "set mode land...");
    setMode(node, rate, "AUTO.LAND");
    waitLand(node, rate);
    RCLCPP_INFO(node->get_logger(), "================= Mission Complete =================");
    rclcpp::shutdown();
    return 0;
}
