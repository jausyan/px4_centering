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

float lateral_tolerance;
float vertical_tolerance;
int min_points_vertical;
int min_points_horizontal;
float max_time = 30.0;  

PathConfig path_config;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate rate(RATE);
    auto node = std::make_shared<DroneController>();

    node->declare_parameter<std::vector<double>>("path_following.waypoint_x", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("path_following.waypoint_y", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("path_following.waypoint_z", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("path_following.waypoint_yaw", std::vector<double>{});
    node->declare_parameter<float>("path_following.speed", 10.0);
    node->declare_parameter<float>("path_following.tolerance", 0.75);
    node->declare_parameter<bool>("path_following.auto_heading", true);

    node->declare_parameter<bool>("path_following.enable_bspline_smoothing", true);
    node->declare_parameter<float>("path_following.bspline_smoothness", 0.0001f);
    node->declare_parameter<int>("path_following.bspline_degree", 3);
    node->declare_parameter<int>("path_following.num_interpolation_points", 200);

    node->declare_parameter<bool>("path_following.enable_rdp_simplification", true);
    node->declare_parameter<float>("path_following.rdp_epsilon", 0.05f);

    node->declare_parameter<float>("path_following.slow_down_distance", 4.0f);
    node->declare_parameter<bool>("path_following.enable_lateral_compensation", true);
    node->declare_parameter<float>("path_following.lateral_compensation_factor", 15.0f);
    
    node->declare_parameter<float>("gate_centering.takeoff_altitude", 2.0);
    node->declare_parameter<float>("gate_centering.forward_distance", 3.0);
    node->declare_parameter<float>("gate_centering.gate_width", 1.5);
    node->declare_parameter<float>("gate_centering.centering_tolerance", 0.15);
    node->declare_parameter<float>("gate_centering.waypoint_tolerance", 0.3);
    node->declare_parameter<std::string>("gate_centering.control_mode", "velocity"); 
    node->declare_parameter<float>("gate_centering.max_velocity", 0.2);  
    node->declare_parameter<float>("gate_centering.proportional_gain", 0.5);  
    node->declare_parameter<float>("gate_centering.rotation_angle", 90.0);  

    node->declare_parameter<float>("gate_centering.detection_range_min", 1.5);
    node->declare_parameter<float>("gate_centering.detection_range_max", 2.5);
    node->declare_parameter<float>("gate_centering.roi_lateral", 2.0);
    node->declare_parameter<float>("gate_centering.cluster_epsilon", 0.15);
    node->declare_parameter<int>("gate_centering.min_cluster_points", 10);
    node->declare_parameter<float>("gate_centering.min_pole_height", 0.5);
    node->declare_parameter<float>("gate_centering.target_height", 0.75);
    
    node->get_parameter("gate_centering.takeoff_altitude", takeoff_altitude);
    node->get_parameter("gate_centering.forward_distance", forward_distance);
    node->get_parameter("gate_centering.gate_width", gate_width);
    node->get_parameter("gate_centering.centering_tolerance", centering_tolerance);
    node->get_parameter("gate_centering.waypoint_tolerance", waypoint_tolerance);
    node->get_parameter("gate_centering.control_mode", control_mode);
    node->get_parameter("gate_centering.max_velocity", max_velocity);
    node->get_parameter("gate_centering.proportional_gain", proportional_gain);
    node->get_parameter("gate_centering.rotation_angle", rotation_angle);
    rotation_radians = rotation_angle * M_PI / 180.0;
    
    std::vector<double> wp_x, wp_y, wp_z, wp_yaw;
    node->get_parameter("path_following.waypoint_x", wp_x);
    node->get_parameter("path_following.waypoint_y", wp_y);
    node->get_parameter("path_following.waypoint_z", wp_z);
    node->get_parameter("path_following.waypoint_yaw", wp_yaw);

    if (wp_x.size() == wp_y.size() && wp_y.size() == wp_z.size() && wp_z.size() == wp_yaw.size() && !wp_x.empty()) {
        for (size_t i = 0; i < wp_x.size(); i++) {
            PathWaypoint wp;
            wp.x = static_cast<float>(wp_x[i]);
            wp.y = static_cast<float>(wp_y[i]);
            wp.z = static_cast<float>(wp_z[i]);
            wp.yaw = static_cast<float>(wp_yaw[i]);
            path_config.waypoints.push_back(wp);
        }
        RCLCPP_INFO(node->get_logger(), "Loaded %zu waypoints from config", path_config.waypoints.size());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Waypoint array size mismatch!");
        RCLCPP_ERROR(node->get_logger(), "  waypoint_x: %zu elements", wp_x.size());
        RCLCPP_ERROR(node->get_logger(), "  waypoint_y: %zu elements", wp_y.size());
        RCLCPP_ERROR(node->get_logger(), "  waypoint_z: %zu elements", wp_z.size());
        RCLCPP_ERROR(node->get_logger(), "  waypoint_yaw: %zu elements", wp_yaw.size());
        RCLCPP_ERROR(node->get_logger(), "Please check config.yaml - all arrays must have same size!");
    }
    
    node->get_parameter("path_following.speed", path_config.speed);
    node->get_parameter("path_following.tolerance", path_config.tolerance);
    node->get_parameter("path_following.auto_heading", path_config.auto_heading);
    
    // B-Spline smoothing
    node->get_parameter("path_following.enable_bspline_smoothing", path_config.enable_bspline_smoothing);
    node->get_parameter("path_following.bspline_smoothness", path_config.bspline_smoothness);
    node->get_parameter("path_following.bspline_degree", path_config.bspline_degree);
    node->get_parameter("path_following.num_interpolation_points", path_config.num_interpolation_points);
    
    // RDP simplification
    node->get_parameter("path_following.enable_rdp_simplification", path_config.enable_rdp_simplification);
    node->get_parameter("path_following.rdp_epsilon", path_config.rdp_epsilon);
    
    // Velocity control
    node->get_parameter("path_following.slow_down_distance", path_config.slow_down_distance);
    node->get_parameter("path_following.enable_lateral_compensation", path_config.enable_lateral_compensation);
    node->get_parameter("path_following.lateral_compensation_factor", path_config.lateral_compensation_factor);
    
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

    // MISSION START HERE
    
    geometry_msgs::msg::PoseStamped posee;
    initFrame(node, posee);
    
    while(rclcpp::ok() && node->getCurrentLocalPose().pose.position.z == 0.0) {
        RCLCPP_INFO(node->get_logger(), "Wait local pose data...");
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    // RCLCPP_INFO(node->get_logger(), "Wait lidar data...");
    // while(rclcpp::ok() && node->getCurrentLaserScan().ranges.empty()) {
    //     RCLCPP_INFO(node->get_logger(), "Waiting lidar scan...");
    //     rclcpp::spin_some(node);
    //     rate.sleep();
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }
    // RCLCPP_INFO(node->get_logger(), "Lidar data received!");
    
    RCLCPP_INFO(node->get_logger(), "Step 1: Taking off %.2f meters...", takeoff_altitude);
    takeoff(node, rate, posee, takeoff_altitude);
    holdPosition(node, rate, posee, 2.0);

    bool centering_status = false;

    centeringGateLivoxSimple(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, max_time);
    if (centering_status) {
        RCLCPP_INFO(node->get_logger(), "============= CENTERING DONE.. MAJUUU.... ===============");
        RCLCPP_INFO(node->get_logger(), "MOVING FORWARD %.2f METERS...", forward_distance);
        executeLocalWaypointMoveVelocity(node, rate, posee, 4.0, 0.0, 0.0, 0.0, 1.0, waypoint_tolerance);
        holdPosition(node, rate, posee, 1.0);
    } else {
        RCLCPP_WARN(node->get_logger(), "GAGAL CENTERING LANDING....!!!");
        setMode(node, rate, "AUTO.LAND");
        waitLand(node, rate);
    }
    
    RCLCPP_INFO(node->get_logger(), "START PATH FOLLOWING...");
    followPathFromConfig(node, rate, posee, path_config);
    RCLCPP_INFO(node->get_logger(), "Path complete");

    RCLCPP_INFO(node->get_logger(), "LANDINGGG....!!!");
    setMode(node, rate, "AUTO.LAND");
    waitLand(node, rate);
    
    RCLCPP_INFO(node->get_logger(), "================= Mission Complete =================");
    
    rclcpp::shutdown();
    return 0;
}
