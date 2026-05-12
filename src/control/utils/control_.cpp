#include "control_.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <algorithm>

bool payloadDetected(const std::shared_ptr<DroneController>&node){
    /**
     * Check if the payload is detected by verifying if its position is not at the origin (0,0,0).]
     * Since PointStamped default constructor initializes position to (0,0,0),
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * 
     * returns:
     * - true if the payload is detected, false otherwise.
     */
    geometry_msgs::msg::PoseStamped payload_pose = node->getCurrentPosePayload();
    return (payload_pose.pose.position.x != 0.000 && payload_pose.pose.position.y != 0.000 && payload_pose.pose.position.z != 0.000);
}

bool emberDetected(const std::shared_ptr<DroneController>&node){
    /**
     * Check if the ember is detected by verifying if its position is not at the origin (0,0,0).
     * Since PointStamped default constructor initializes position to (0,0,0),
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * 
     * returns:
     * - true if the ember is detected, false otherwise.
     */
    geometry_msgs::msg::PoseStamped ember_pose = node->getCurrentPoseEmber();
    return (ember_pose.pose.position.x != 0.000 && ember_pose.pose.position.y != 0.000 && ember_pose.pose.position.z != 0.000);
}

float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool bunderDetected(const std::shared_ptr<DroneController>&node){
    /**
     * Check if the ember is detected by verifying if its position is not at the origin (0,0,0).
     * Since PointStamped default constructor initializes position to (0,0,0),
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * 
     * returns:
     * - true if the ember is detected, false otherwise.
     */
    geometry_msgs::msg::PoseStamped bunder_pose = node->getCurrentPoseBunder();
    return (bunder_pose.pose.position.x != 0.000 && bunder_pose.pose.position.y != 0.000 && bunder_pose.pose.position.z != 0.000);
}

bool hasReplied(const std::shared_ptr<DroneController>&node, auto& result){
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
    return rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS;
}

void initFrame(const std::shared_ptr<DroneController>& node, geometry_msgs::msg::PoseStamped &pose) {
    /**
     * Initialize the starting pose 'node' with the current position of the drone and reset payload pose.
     * This is used to reset posee or agg_pose (reference to where the last drone position when a control
     * function is called, it will be modified to the last drone position when the function returns.)
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - pose: reference to the geometry_msgs::msg::PoseStamped object to be initialized.
     */
    geometry_msgs::msg::PoseStamped last_payload_pose = node->getLastPayloadPose();
    setPosition(last_payload_pose,0,0,0);
    node->setLastPayloadPose(last_payload_pose);
    for(int i = 0; i < 30 && rclcpp::ok(); i++) {
        geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
        setPosition(pose, curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z);
        pose.pose.orientation = curr_pose.pose.orientation;
        rclcpp::spin_some(node); rclcpp::Rate(RATE).sleep();
    }
}


void setParam(const std::shared_ptr<DroneController>&node, const std::string &id, int integer_value) {\
    /**
     * Set a parameter on the drone controller node.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - id: string identifier for the parameter to be set.
     * - integer_value: integer value to set for the parameter.
     */
    auto set_param_req = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();

    set_param_req->force_set = true;
    set_param_req->param_id = id;
    set_param_req->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    set_param_req->value.integer_value = integer_value;

    RCLCPP_INFO(node->get_logger(), "Sekkk nunggu service param_set");
    node->waitForSetParamService();

    if (node->isSetParamServiceReady()) {
        auto result_future = node->setParam_(set_param_req);

        if (hasReplied(node, result_future)) {
            auto result = result_future.get();
            if (result->success) { RCLCPP_INFO(node->get_logger(), "GGWP Param berubah haruse"); } else { RCLCPP_ERROR(node->get_logger(), "Nooo gagal ngubah param"); }
        } else { RCLCPP_ERROR(node->get_logger(), "Timeout / gagal mendapatkan respon dari service"); }
    } else { RCLCPP_ERROR(node->get_logger(), "Set param service tidak tersedia setelah menunggu"); }
}

void setMode(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, const std::string mode) {
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = mode;   
    
    RCLCPP_INFO(node->get_logger(), "Attempting to set mode to %s...", mode.c_str());
    
    if (node->isSetModeServiceReady()) {   
        auto set_mode_result = node->setMode_(set_mode_request);   
        if (rclcpp::spin_until_future_complete(node, set_mode_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = set_mode_result.get();
            if (result->mode_sent) {
                RCLCPP_INFO(node->get_logger(), "Mode set to %s", mode.c_str());
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to set mode to %s", mode.c_str());
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to call SetMode service");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "SetMode service not ready");
    }
    rate.sleep();
}

void takeoff(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt) {
    /**
     * Initiate the takeoff procedure for the drone. Set mode to OFFBOARD, arm the drone, and send the takeoff command.
     * For PX4: Stream setpoint at high rate BEFORE switching to OFFBOARD mode
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - takeoff_alt: float value representing the desired takeoff altitude, this is based on the rangefinder reading.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     * - rel_alt: float value representing the relative altitude to be maintained at the correct rangefinder reading.
     */
    
    RCLCPP_INFO(node->get_logger(), "=== TAKEOFF ===");
    RCLCPP_INFO(node->get_logger(), "Target altitude: %.2f meters", takeoff_alt);
    
    // Get current position - NO OFFSET CORRECTION, use raw local position Z
    geometry_msgs::msg::PoseStamped target_pose = node->getCurrentLocalPose();
    // Set target altitude directly from local position z + desired takeoff altitude
    target_pose.pose.position.z = target_pose.pose.position.z + takeoff_alt;
    
    RCLCPP_INFO(node->get_logger(), "Current Z: %.3f m, Target Z: %.3f m", 
                node->getCurrentLocalPose().pose.position.z, target_pose.pose.position.z);
    
    // PX4 CRITICAL: Stream setpoints BEFORE switching to OFFBOARD mode
    // Send at least 100 setpoints at 20Hz (2 seconds) before mode switch
    rclcpp::Rate fast_rate(20.0); // 20 Hz for PX4
    for(int i = 0; i < 40 && rclcpp::ok(); i++) {
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Setting OFFBOARD mode...");
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "OFFBOARD";   
    
    if (node->isSetModeServiceReady()) {   
        auto set_mode_result = node->setMode_(set_mode_request);   
        if (rclcpp::spin_until_future_complete(node, set_mode_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = set_mode_result.get();
            if (result->mode_sent) {
                RCLCPP_INFO(node->get_logger(), "OFFBOARD mode set");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to set OFFBOARD mode");
                return;
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to call SetMode service");
            return;
        }
    }
    
    // Continue streaming while waiting
    for(int i = 0; i < 10 && rclcpp::ok(); i++) {
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Arming vehicle...");
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;

    if (node->isArmingServiceReady()) {
        auto arm_result = node->arm_(arm_request);
        if (rclcpp::spin_until_future_complete(node, arm_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = arm_result.get();
            if (result->success) {
                RCLCPP_INFO(node->get_logger(), "Vehicle armed");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to arm: %s", result->result ? "true" : "false");
                return;
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to arm vehicle");
            return;
        }
    }

    RCLCPP_INFO(node->get_logger(), "=== TAKEOFF CONFIRMATION ===");
    RCLCPP_INFO(node->get_logger(), "takeoff 1 if yes ?");  
    int confirmation;
    std::cin >> confirmation;
    
    if (confirmation != 1) {
        RCLCPP_WARN(node->get_logger(), "Takeoff cancelled by user");
        return;
    }
    
    // Continue streaming during arm
    for(int i = 0; i < 10 && rclcpp::ok(); i++) {
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Ascending to %.2f meters...", takeoff_alt);
    
    // For PX4 OFFBOARD: Just stream position setpoints, no separate takeoff command needed
    // The drone will automatically follow the z position setpoint
    auto start_time = node->now();
    const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(30.0);
    bool reached = false;
    double current_alt = 0.0;

    while (rclcpp::ok()) {
        // Keep streaming setpoint at high rate (CRITICAL for PX4 OFFBOARD)
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        
        // Get current altitude - NO OFFSET, use raw local position Z
        current_alt = node->getCurrentLocalPose().pose.position.z;
        
        if (fmod((node->now() - start_time).seconds(), 0.1) < 0.05) { // Log every ~1 second
            RCLCPP_INFO(node->get_logger(), "Altitude: %.2f m / %.2f m (target)", 
                        current_alt, target_pose.pose.position.z);
        }

        // Check if reached target altitude (direct comparison without offset)
        if (fabs(target_pose.pose.position.z - current_alt) < 0.15) {
            reached = true; 
            break;
        }

        if ((node->now() - start_time) > timeout_duration) {
            RCLCPP_WARN(node->get_logger(), "Timeout waiting to reach takeoff altitude");
            break;
        }

        rclcpp::spin_some(node);
        fast_rate.sleep(); // Keep streaming at 20Hz
    }

    if (reached) {
        RCLCPP_INFO(node->get_logger(), "Reached takeoff altitude! (%.2f m)", current_alt);
    } else {
        RCLCPP_WARN(node->get_logger(), "Did not reach takeoff altitude within timeout");
    }
    
    rate.sleep();
    posee.pose.position.z = node->getCurrentLocalPose().pose.position.z;
}

// ======================= TAKEOFF PROCEDURE ===================
// ALT OFFSET TAKEOFF
// =============================================================
// void takeoff(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt) {
//     /**
//      * Initiate the takeoff procedure for the drone. Set mode to OFFBOARD, arm the drone, and send the takeoff command.
//      * For PX4: Stream setpoint at high rate BEFORE switching to OFFBOARD mode
//      * parameters:
//      * - node: shared pointer to the DroneController node instance.
//      * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
//      * - takeoff_alt: float value representing the desired takeoff altitude, this is based on the rangefinder reading.
//      * 
//      * returns:
//      * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
//      * - rel_alt: float value representing the relative altitude to be maintained at the correct rangefinder reading.
//      */
    
//     // ========== ALTITUDE OFFSET CALIBRATION ==========
//     // Check ground altitude and calculate offset to compensate for sensor drift
//     RCLCPP_INFO(node->get_logger(), "=== ALTITUDE CALIBRATION ===");
    
//     // Take multiple samples to get accurate ground level
//     const int num_samples = 10;
//     float altitude_sum = 0.0f;
//     rclcpp::Rate sample_rate(10.0); // 10 Hz sampling
    
//     for (int i = 0; i < num_samples && rclcpp::ok(); i++) {
//         float current_z = node->getCurrentLocalPose().pose.position.z;
//         altitude_sum += current_z;
//         rclcpp::spin_some(node);
//         sample_rate.sleep();
//     }
    
//     float ground_altitude_offset = altitude_sum / num_samples;
    
//     // Store the offset in DroneController for use by other functions
//     node->setGroundAltitudeOffset(ground_altitude_offset);
    
//     RCLCPP_INFO(node->get_logger(), "Ground altitude: %.3f m", ground_altitude_offset);
    
//     if (fabs(ground_altitude_offset) > 0.05f) {
//         RCLCPP_WARN(node->get_logger(), "AWAS NAIK! COBA SET O");
//     } else {
//         RCLCPP_INFO(node->get_logger(), "ALT set 0 (offset < 0.05m)");
//     }
    
//     // USER CONFIRMATION
//     // RCLCPP_INFO(node->get_logger(), "=== TAKEOFF CONFIRMATION ===");
//     // RCLCPP_INFO(node->get_logger(), "takeoff 1 if yes ?");  
//     // int confirmation;
//     // std::cin >> confirmation;
    
//     // if (confirmation != 1) {
//     //     RCLCPP_WARN(node->get_logger(), "Takeoff cancelled by user");
//     //     return;
//     // }
    
//     // Get current position and apply offset compensation
//     geometry_msgs::msg::PoseStamped target_pose = node->getCurrentLocalPose();
//     // Set target altitude: takeoff_alt + offset to compensate for ground drift
//     target_pose.pose.position.z = takeoff_alt + ground_altitude_offset;
    
//     RCLCPP_INFO(node->get_logger(), "=== TAKEOFF ===");
//     RCLCPP_INFO(node->get_logger(), "taking off...");
    
//     // PX4 CRITICAL: Stream setpoints BEFORE switching to OFFBOARD mode
//     // Send at least 100 setpoints at 20Hz (2 seconds) before mode switch
//     rclcpp::Rate fast_rate(20.0); // 20 Hz for PX4
//     for(int i = 0; i < 40 && rclcpp::ok(); i++) {
//         target_pose.header.stamp = node->now();
//         node->publishLocalPosition(target_pose);
//         rclcpp::spin_some(node);
//         fast_rate.sleep();
//     }
    
//     RCLCPP_INFO(node->get_logger(), "Setting OFFBOARD mode...");
//     auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
//     set_mode_request->custom_mode = "OFFBOARD";   
    
//     if (node->isSetModeServiceReady()) {   
//         auto set_mode_result = node->setMode_(set_mode_request);   
//         if (rclcpp::spin_until_future_complete(node, set_mode_result) == rclcpp::FutureReturnCode::SUCCESS) {
//             auto result = set_mode_result.get();
//             if (result->mode_sent) {
//                 RCLCPP_INFO(node->get_logger(), "OFFBOARD mode set");
//             } else {
//                 RCLCPP_ERROR(node->get_logger(), "Failed to set OFFBOARD mode");
//                 return;
//             }
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "Failed to call SetMode service");
//             return;
//         }
//     }
    
//     // Continue streaming while waiting
//     for(int i = 0; i < 10 && rclcpp::ok(); i++) {
//         target_pose.header.stamp = node->now();
//         node->publishLocalPosition(target_pose);
//         rclcpp::spin_some(node);
//         fast_rate.sleep();
//     }
    
//     RCLCPP_INFO(node->get_logger(), "Arming vehicle...");
//     auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
//     arm_request->value = true;

//     if (node->isArmingServiceReady()) {
//         auto arm_result = node->arm_(arm_request);
//         if (rclcpp::spin_until_future_complete(node, arm_result) == rclcpp::FutureReturnCode::SUCCESS) {
//             auto result = arm_result.get();
//             if (result->success) {
//                 RCLCPP_INFO(node->get_logger(), "Vehicle armed");
//             } else {
//                 RCLCPP_ERROR(node->get_logger(), "Failed to arm: %s", result->result ? "true" : "false");
//                 return;
//             }
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "Failed to arm vehicle");
//             return;
//         }
//     }

//     RCLCPP_INFO(node->get_logger(), "=== TAKEOFF CONFIRMATION ===");
//     RCLCPP_INFO(node->get_logger(), "takeoff 1 if yes ?");  
//     int confirmation;
//     std::cin >> confirmation;
    
//     if (confirmation != 1) {
//         RCLCPP_WARN(node->get_logger(), "Takeoff cancelled by user");
//         return;
//     }
    
//     // Continue streaming during arm
//     for(int i = 0; i < 10 && rclcpp::ok(); i++) {
//         target_pose.header.stamp = node->now();
//         node->publishLocalPosition(target_pose);
//         rclcpp::spin_some(node);
//         fast_rate.sleep();
//     }

//     RCLCPP_INFO(node->get_logger(), "Ascending to %.2f meters (relative to ground)...", takeoff_alt);
    
//     // For PX4 OFFBOARD: Just stream position setpoints, no separate takeoff command needed
//     // The drone will automatically follow the z position setpoint
//     auto start_time = node->now();
//     const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(30.0);
//     bool reached = false;

//     while (rclcpp::ok()) {
//         // Keep streaming setpoint at high rate (CRITICAL for PX4 OFFBOARD)
//         target_pose.header.stamp = node->now();
//         node->publishLocalPosition(target_pose);
        
//         // Get current altitude (with offset)
//         double current_alt_raw = node->getCurrentLocalPose().pose.position.z;
//         // Calculate actual altitude relative to ground (compensate offset)
//         double current_alt_relative = current_alt_raw - ground_altitude_offset;
        
//         if (fmod((node->now() - start_time).seconds(), 1.0) < 0.05) { // Log every ~1 second
//             RCLCPP_INFO(node->get_logger(), "Altitude: %.2f m / %.2f m (raw: %.2f m, offset: %.2f m)", 
//                         current_alt_relative, takeoff_alt, current_alt_raw, ground_altitude_offset);
//         }

//         // Check if reached target altitude (compare relative altitude)
//         if (fabs(takeoff_alt - current_alt_relative) < 0.15) {
//             reached = true; 
//             break;
//         }

//         if ((node->now() - start_time) > timeout_duration) {
//             RCLCPP_WARN(node->get_logger(), "Timeout waiting to reach takeoff altitude");
//             break;
//         }

//         rclcpp::spin_some(node);
//         fast_rate.sleep(); // Keep streaming at 20Hz
//     }

//     if (reached) {
//         RCLCPP_INFO(node->get_logger(), "Reached takeoff altitude! (%.2f m relative to ground)", takeoff_alt);
//     } else {
//         RCLCPP_WARN(node->get_logger(), "Did not reach takeoff altitude within timeout");
//     }
    
//     rate.sleep();
//     posee.pose.position.z = node->getCurrentLocalPose().pose.position.z;
// }


void executeLocalWaypointMoveVelocity(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                               geometry_msgs::msg::PoseStamped &posee, float forward_x, float left_y, 
                               float up_z, float yaw_angle, float max_velocity, float tolerance) {
    /**
     * Execute a local waypoint movement using VELOCITY CONTROL for smooth, controlled movement
     * Movement is relative to the current drone's heading (body frame)
     * 
     * Unlike executeLocalWaypointMove() which uses position control and can cause sudden movements,
     * this function sends velocity setpoints continuously at max_velocity for smooth motion.
     * 
     * Time taken = distance / max_velocity
     * Example: 2 meters at 0.2 m/s = 10 seconds
     */
    
    // Get current position and heading
    geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
    float current_yaw = getHeading(current_pose.pose.orientation);
    
    RCLCPP_INFO(node->get_logger(), "=== Velocity Control Movement ===");
    RCLCPP_INFO(node->get_logger(), "Current heading: %.2f degrees", current_yaw * 180.0 / M_PI);
    RCLCPP_INFO(node->get_logger(), "Body frame: forward=%.2f m, left=%.2f m, up=%.2f m", forward_x, left_y, up_z);
    RCLCPP_INFO(node->get_logger(), "Max velocity: %.2f m/s", max_velocity);
    
    // Transform body-frame movement to global frame using current heading
    float global_x = forward_x * cos(current_yaw) - left_y * sin(current_yaw);
    float global_y = forward_x * sin(current_yaw) + left_y * cos(current_yaw);
    
    // Calculate target position in global frame (using relative altitude)
    float target_x = current_pose.pose.position.x + global_x;
    float target_y = current_pose.pose.position.y + global_y;
    float current_relative_z = node->getRelativeAltitude();
    float target_z = current_relative_z + up_z;
    
    // print log every 0.2 seconds
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 200, "Target global: x=%.2f, y=%.2f, z=%.2f", target_x, target_y, target_z);
    
    // Calculate total distance to travel
    float total_distance = sqrt(global_x * global_x + global_y * global_y + up_z * up_z);
    float estimated_time = total_distance / max_velocity;
    
    RCLCPP_INFO(node->get_logger(), "Total distance: %.2f m, Estimated time: %.1f seconds", 
                total_distance, estimated_time);
    
    // PX4 OFFBOARD: Use fast rate for continuous streaming
    rclcpp::Rate fast_rate(20.0); // 20Hz
    
    // Main control loop
    while (rclcpp::ok()) {
        current_pose = node->getCurrentLocalPose();
        
        // Calculate current distance to target (using relative altitude for Z)
        float dx = target_x - current_pose.pose.position.x;
        float dy = target_y - current_pose.pose.position.y;
        float current_relative_z_loop = node->getRelativeAltitude();
        float dz = target_z - current_relative_z_loop;
        float current_distance = sqrt(dx * dx + dy * dy + dz * dz);
        
        // Check if we've reached the target
        if (current_distance < tolerance) {
            RCLCPP_INFO(node->get_logger(), "Target reached! Distance: %.3f m", current_distance);
            break;
        }
        
        // Calculate velocity direction (unit vector towards target)
        geometry_msgs::msg::TwistStamped vel_cmd;
        vel_cmd.header.stamp = node->now();
        vel_cmd.header.frame_id = "map";
        
        // Normalize direction and multiply by max_velocity
        vel_cmd.twist.linear.x = (dx / current_distance) * max_velocity;
        vel_cmd.twist.linear.y = (dy / current_distance) * max_velocity;
        
        // Add deadband for Z to prevent altitude drift
        if (std::abs(dz) > 0.15f) {  // Only control altitude if error > 15cm
            vel_cmd.twist.linear.z = (dz / current_distance) * max_velocity;
        } else {
            vel_cmd.twist.linear.z = 0.0;  // Hold altitude when close enough
        }
        
        // Publish velocity command
        node->publishLocalVelocity(vel_cmd);
        
        // Log progress (throttled to 1 Hz)
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                           "Distance remaining: %.3f m | Velocity: vx=%.2f, vy=%.2f, vz=%.2f", 
                           current_distance, 
                           vel_cmd.twist.linear.x, 
                           vel_cmd.twist.linear.y, 
                           vel_cmd.twist.linear.z);
        
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    // Stop the drone by sending zero velocity
    geometry_msgs::msg::TwistStamped stop_vel;
    stop_vel.header.stamp = node->now();
    stop_vel.header.frame_id = "map";
    stop_vel.twist.linear.x = 0.0;
    stop_vel.twist.linear.y = 0.0;
    stop_vel.twist.linear.z = 0.0;
    node->publishLocalVelocity(stop_vel);
    
    RCLCPP_INFO(node->get_logger(), "Movement complete! Stopping...");
    
    // Update posee to final position
    posee = node->getCurrentLocalPose();
    
    RCLCPP_INFO(node->get_logger(), "Final position: x=%.2f, y=%.2f, z=%.2f", 
                posee.pose.position.x, posee.pose.position.y, posee.pose.position.z);
}

// void executeLocalWaypointMoveVelocity(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
//                                geometry_msgs::msg::PoseStamped &posee, float forward_x, float left_y, 
//                                float up_z, float yaw_angle, float max_velocity, float tolerance) {
//     /**
//      * Execute a local waypoint movement using VELOCITY CONTROL for smooth, controlled movement
//      * Movement is relative to the current drone's heading (body frame)
//      * 
//      * Unlike executeLocalWaypointMove() which uses position control and can cause sudden movements,
//      * this function sends velocity setpoints continuously at max_velocity for smooth motion.
//      * 
//      * Time taken = distance / max_velocity
//      * Example: 2 meters at 0.2 m/s = 10 seconds
//      */
    
//     // Get current position and heading
//     geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//     float current_yaw = getHeading(current_pose.pose.orientation);
    
//     RCLCPP_INFO(node->get_logger(), "=== Velocity Control Movement ===");
//     RCLCPP_INFO(node->get_logger(), "Current heading: %.2f degrees", current_yaw * 180.0 / M_PI);
//     RCLCPP_INFO(node->get_logger(), "Body frame: forward=%.2f m, left=%.2f m, up=%.2f m", forward_x, left_y, up_z);
//     RCLCPP_INFO(node->get_logger(), "Max velocity: %.2f m/s", max_velocity);
    
//     // Transform body-frame movement to global frame using current heading
//     float global_x = forward_x * cos(current_yaw) - left_y * sin(current_yaw);
//     float global_y = forward_x * sin(current_yaw) + left_y * cos(current_yaw);
    
//     // Calculate target position in global frame
//     float target_x = current_pose.pose.position.x + global_x;
//     float target_y = current_pose.pose.position.y + global_y;
//     float target_z = current_pose.pose.position.z + up_z;
    
//     RCLCPP_INFO(node->get_logger(), "Target global: x=%.2f, y=%.2f, z=%.2f", target_x, target_y, target_z);
    
//     // Calculate total distance to travel
//     float total_distance = sqrt(global_x * global_x + global_y * global_y + up_z * up_z);
//     float estimated_time = total_distance / max_velocity;
    
//     RCLCPP_INFO(node->get_logger(), "Total distance: %.2f m, Estimated time: %.1f seconds", 
//                 total_distance, estimated_time);
    
//     // PX4 OFFBOARD: Use fast rate for continuous streaming
//     rclcpp::Rate fast_rate(20.0); // 20Hz
    
//     // Main control loop
//     while (rclcpp::ok()) {
//         current_pose = node->getCurrentLocalPose();
        
//         // Calculate current distance to target
//         float dx = target_x - current_pose.pose.position.x;
//         float dy = target_y - current_pose.pose.position.y;
//         float dz = target_z - current_pose.pose.position.z;
//         float current_distance = sqrt(dx * dx + dy * dy + dz * dz);
        
//         // Check if we've reached the target
//         if (current_distance < tolerance) {
//             RCLCPP_INFO(node->get_logger(), "Target reached! Distance: %.3f m", current_distance);
//             break;
//         }
        
//         // Calculate velocity direction (unit vector towards target)
//         geometry_msgs::msg::TwistStamped vel_cmd;
//         vel_cmd.header.stamp = node->now();
//         vel_cmd.header.frame_id = "map";
        
//         // Normalize direction and multiply by max_velocity
//         vel_cmd.twist.linear.x = (dx / current_distance) * max_velocity;
//         vel_cmd.twist.linear.y = (dy / current_distance) * max_velocity;
//         vel_cmd.twist.linear.z = (dz / current_distance) * max_velocity;
        
//         // Publish velocity command
//         node->publishLocalVelocity(vel_cmd);
        
//         // Log progress (throttled to 1 Hz)
//         RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                            "Distance remaining: %.3f m | Velocity: vx=%.2f, vy=%.2f, vz=%.2f", 
//                            current_distance, 
//                            vel_cmd.twist.linear.x, 
//                            vel_cmd.twist.linear.y, 
//                            vel_cmd.twist.linear.z);
        
//         rclcpp::spin_some(node);
//         fast_rate.sleep();
//     }
    
//     // Stop the drone by sending zero velocity
//     geometry_msgs::msg::TwistStamped stop_vel;
//     stop_vel.header.stamp = node->now();
//     stop_vel.header.frame_id = "map";
//     stop_vel.twist.linear.x = 0.0;
//     stop_vel.twist.linear.y = 0.0;
//     stop_vel.twist.linear.z = 0.0;
//     node->publishLocalVelocity(stop_vel);
    
//     RCLCPP_INFO(node->get_logger(), "Movement complete! Stopping...");
    
//     // Update posee to final position
//     posee = node->getCurrentLocalPose();
    
//     RCLCPP_INFO(node->get_logger(), "Final position: x=%.2f, y=%.2f, z=%.2f", 
//                 posee.pose.position.x, posee.pose.position.y, posee.pose.position.z);
// }

void takeoff_no_confirm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float takeoff_alt) {
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
    // auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    // set_mode_request->custom_mode = "GUIDED";   
    
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;

    // if (node->isSetModeServiceReady()) {   
    //     auto set_mode_result = node->setMode_(set_mode_request);   
    //     if (rclcpp::spin_until_future_complete(node, set_mode_result) == rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_INFO(node->get_logger(), "Set mode to GUIDED");
    //     } else {
    //         RCLCPP_ERROR(node->get_logger(), "Failed to call SetMode service");
    //     }
    // }
    // rate.sleep();

    if (node->isArmingServiceReady()) {
        auto arm_result = node->arm_(arm_request);
        if (rclcpp::spin_until_future_complete(node, arm_result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Vehicle armed");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to arm vehicle");
        }
    }
    rate.sleep();

    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    takeoff_request->altitude = takeoff_alt + node->getCurrentRelAlt().data;
    if(takeoff_request->altitude <= takeoff_alt - node->getCurrentRangefinder().range)
    {
        takeoff_request->altitude = takeoff_alt;
    }

    if (node->isTakeoffServiceReady()) {
        for (int i = 0; i<5; ++i)
        {
            auto takeoff_result = node->takeoff_(takeoff_request);
            if (rclcpp::spin_until_future_complete(node, takeoff_result) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Takeoff command sent, request: %f, range: %f, rel_alt: %f, z: %f",
                    takeoff_request->altitude, node->getCurrentRangefinder().range, node->getCurrentRelAlt().data, node->getCurrentLocalPose().pose.position.z);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to send takeoff command");
            }
        }
    }

    RCLCPP_INFO(node->get_logger(), "Waiting to reach altitude %.2f m (timeout: 5s)...", takeoff_alt);
    auto start_time = node->now();
    const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(5.0);

    bool reached = false;

    while (rclcpp::ok()) {
        double alt = node->getCurrentRangefinder().range;
        RCLCPP_INFO(node->get_logger(), "Current altitude: %.2f m", alt);

        if ( 0.1 >= abs(0.55 - alt)) {
            reached = true; 
            break;
        }

        if ((node->now() - start_time) > timeout_duration) {
            RCLCPP_WARN(node->get_logger(), "Timeout waiting to reach takeoff altitude");
            break;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    if (reached) {
        RCLCPP_INFO(node->get_logger(), "Reached takeoff altitude!");
    } else {
        RCLCPP_WARN(node->get_logger(), "Did not reach takeoff altitude within timeout");
    }

    rate.sleep();
    posee.pose.position.z = node->getCurrentLocalPose().pose.position.z;
}

// void stabilize(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &target_pose, float max_time) {
//     /**
//      * Wait for the drone to stably hover at the target pose while constantly updating pose and heading to mitigate drift.
//      * 
//      * parameters:
//      * - node: shared pointer to the DroneController node instance.
//      * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
//      * - target_pose: reference to the geometry_msgs::msg::PoseStamped object representing the target pose for stabilization.
//      */
//     geometry_msgs::msg::PoseStamped new_pose;
//     rclcpp::Time start_time = node->now();

//     while (rclcpp::ok()) {
//         rclcpp::Time current_time = node->now();
//         float elapsed_time = (current_time - start_time).seconds();
//         // if (isNear(curr_pose.pose.position, target_pose.pose.position.x, target_pose.pose.position.y, 0.05) && (abs(getHeading(curr_pose.pose.orientation) - getHeading(target_pose.pose.orientation)) < 0.05)){
//         if (isNear(node->getCurrentLocalPose().pose.position, target_pose.pose.position.x, target_pose.pose.position.y, 0.05)){
//             RCLCPP_INFO(node->get_logger(), "++++++++ Stabilized ++++++++++++++++++");
//             break;
//         }

//         if (elapsed_time > max_time) {
//             RCLCPP_WARN(node->get_logger(), "It took too long to stabilize! The drone might be wobbly");
//             break;
//         }

//         setPosition(new_pose, target_pose.pose.position.x, target_pose.pose.position.y, node->getCurrentLocalPose().pose.position.z);
//         setHeading(new_pose.pose.orientation, getHeading(target_pose.pose.orientation));
//         node->publishLocalPosition(new_pose);
//         rclcpp::spin_some(node);
//         rate.sleep();
//     }
// }

void stabilize(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped& posee, float duration){
    /**
     * Wait for a specified duration. In guided mode, the drone should hold the current position when doing nothing.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - duration: float value representing the duration in seconds to hold the position.
     */
    rclcpp::Time start_time = node->now();
    while (rclcpp::ok()) {
        rclcpp::Time current_time = node->now();
        double elapsed_time = (current_time - start_time).seconds();
        
        node->publishLocalPosition(posee);
        RCLCPP_INFO(node->get_logger(), "++++++++++++++++++++ Stabilize ++++++++++++++++++++");
        if (elapsed_time > duration){break;}

        rclcpp::spin_some(node); rate.sleep();
    }
    RCLCPP_INFO(node->get_logger(), "+++++++++++++++++ Selesai Stabilize +++++++++++++++++++");

    posee = node->getCurrentLocalPose();
}

void waitLand(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate){
    /**
     * Wait until the drone has landed by checking its altitude.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     */
    while (rclcpp::ok() && node->getCurrentLocalPose().pose.position.z > 0.1){
        RCLCPP_INFO(node->get_logger(), "current altitude: %f", node->getCurrentLocalPose().pose.position.z);
        rclcpp::spin_some(node); rate.sleep();
    }
}

void land(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate){
    /**
     * Initiate the landing procedure for the drone by sending a land command.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     */
    auto land_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    land_req->altitude = 0;
    if (node->isLandServiceReady()) {
        auto result = node->land_(land_req);
        if (hasReplied(node,result)) { RCLCPP_INFO(node->get_logger(), "land command sent"); waitLand(node,rate);} 
        else { RCLCPP_ERROR(node->get_logger(), "failed to send land command");}
    }
}

void safeLanding(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float descent_rate = 0.3) {
    /**
     * Perform controlled safe landing with gradual descent in OFFBOARD mode.
     * Much safer than using LAND mode which can be too fast.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - posee: reference to current pose (will be updated during descent)
     * - descent_rate: descent speed in m/s (default 0.3 m/s = gentle descent)
     */
    RCLCPP_INFO(node->get_logger(), "=== Starting Safe Landing ===");
    RCLCPP_INFO(node->get_logger(), "Descent rate: %.2f m/s", descent_rate);
    
    rclcpp::Rate fast_rate(20.0); // 20Hz for smooth control
    
    // Get current position
    geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    
    float initial_altitude = current_pose.pose.position.z;
    float target_ground_altitude = 0.05f; // Stop just above ground (5cm)
    
    RCLCPP_INFO(node->get_logger(), "Current altitude: %.2f m", initial_altitude);
    
    // Phase 1: Controlled descent to near ground
    auto descent_start = node->now();
    
    while (rclcpp::ok()) {
        current_pose = node->getCurrentLocalPose();
        float current_alt = current_pose.pose.position.z;
        
        // Check if near ground
        if (current_alt <= target_ground_altitude) {
            RCLCPP_INFO(node->get_logger(), "Near ground (%.2f m), proceeding to disarm...", current_alt);
            break;
        }
        
        // Calculate time-based descent
        float elapsed = (node->now() - descent_start).seconds();
        float desired_alt = initial_altitude - (descent_rate * elapsed);
        
        // Don't go below ground
        if (desired_alt < target_ground_altitude) {
            desired_alt = target_ground_altitude;
        }
        
        // Set target position (maintain x,y, only decrease z)
        target_pose.pose.position.x = current_pose.pose.position.x;
        target_pose.pose.position.y = current_pose.pose.position.y;
        target_pose.pose.position.z = desired_alt;
        target_pose.pose.orientation = current_pose.pose.orientation;
        
        // Stream setpoint
        target_pose.header.stamp = node->now();
        target_pose.header.frame_id = "map";
        node->publishLocalPosition(target_pose);
        
        // Log progress
        if (fmod(elapsed, 1.0) < 0.05) { // Every ~1 second
            RCLCPP_INFO(node->get_logger(), "Landing... Alt: %.2f m -- %.2f m", 
                       current_alt, desired_alt);
        }
        
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    // Phase 2: Hold position briefly at ground level
    RCLCPP_INFO(node->get_logger(), "Holding at ground level for 0.5s...");
    target_pose = node->getCurrentLocalPose();
    target_pose.pose.position.z = 0.0; // Target ground
    
    auto hold_start = node->now();
    while ((node->now() - hold_start).seconds() < 0.5 && rclcpp::ok()) {
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    // // Phase 3: Disarm (safe on ground)
    // RCLCPP_INFO(node->get_logger(), "Disarming...");
    // auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    // arm_request->value = false; // Disarm
    
    // if (node->isArmingServiceReady()) {
    //     auto result = node->arm_(arm_request);
    //     if (hasReplied(node, result)) {
    //         auto response = result.get();
    //         if (response->success) {
    //             RCLCPP_INFO(node->get_logger(), "Disarmed successfully");
    //         } else {
    //             RCLCPP_WARN(node->get_logger(), "Disarm command sent but not confirmed");
    //         }
    //     }
    // }
    
    // Continue streaming at ground for safety
    for (int i = 0; i < 20; i++) { // 1 second
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Safe landing complete!");
    posee = node->getCurrentLocalPose();
}

void Descend(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee){
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
    RCLCPP_INFO(node->get_logger(), "===================== Turun ===================");
    geometry_msgs::msg::TwistStamped vel;
    bool was_descending = false, trying_descending = false;

    vel.twist.linear.x = 0.0;
    vel.twist.linear.y = 0.0;
    vel.twist.linear.z = -0.6;

    while (rclcpp::ok()){
        node->publishLocalVelocity(vel);
        if (node->getCurrentVelocity().twist.linear.z < -0.05 && !trying_descending)  { RCLCPP_INFO(node->get_logger(), "Descend started");trying_descending = true;}
        if (node->getCurrentVelocity().twist.linear.z < -0.35)                        { RCLCPP_INFO(node->get_logger(), "Descending...")  ;was_descending = true;}
        if (was_descending && payloadDetected(node))                                  { node->setLastPayloadPose(node->getCurrentPosePayload());}
        if (abs(node->getCurrentVelocity().twist.linear.z) < 0.055 && was_descending) { RCLCPP_INFO(node->get_logger(), "Landed")         ;break;}
        rclcpp::spin_some(node);rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Latest offset: %f, %f", node->getLastPayloadPose().pose.position.x, node->getLastPayloadPose().pose.position.y);
    posee = node->getCurrentLocalPose();
}

void centering_payload(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, bool &centering_status, float step, float tolerance, float timeout, float max_time, float log_interval, int min_centered_frames) {
    constexpr float frame_width = 640.0f;
    constexpr float frame_height = 480.0f;
    constexpr float center_x = frame_width / 2.0f;
    constexpr float center_y = frame_height / 2.0f;
    if (min_centered_frames < 1) {
        min_centered_frames = 1;
    }
    centering_status = false;
    int centered_frames = 0;
    geometry_msgs::msg::TwistStamped cmd_vel;
    rclcpp::Time start_time = node->now();
    rclcpp::Time last_detection_time = node->now();
    rclcpp::Time last_log_time = node->now() - rclcpp::Duration::from_seconds(log_interval);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        const rclcpp::Time now = node->now();
        const geometry_msgs::msg::PoseStamped payload_pose = node->getCurrentPosePayload();

        const bool pose_recent = payload_pose.header.stamp.nanosec != 0 || payload_pose.header.stamp.sec != 0;
        const bool fresh = pose_recent && (now - payload_pose.header.stamp).seconds() <= timeout;
        const bool detected = fresh && (payload_pose.pose.position.x != 0.0 || payload_pose.pose.position.y != 0.0 || payload_pose.pose.position.z != 0.0);
        if (!detected) {
            centered_frames = 0;
            cmd_vel = zero(cmd_vel);
            node->publishLocalVelocity(cmd_vel);

            if ((now - last_log_time).seconds() >= log_interval) {
                RCLCPP_INFO(node->get_logger(), "NO OBJECT DETECTED");
                last_log_time = now;
            }

            if ((now - last_detection_time).seconds() >= timeout) {
                break;
            }

            if ((now - start_time).seconds() >= max_time) {
                break;
            }

            rate.sleep();
            continue;
        }

        last_detection_time = now;
        const float x = payload_pose.pose.position.x;
        const float y = payload_pose.pose.position.y;
        const float dx = center_x - x;
        const float dy = center_y - y;
        const float center_dist_from_pose = payload_pose.pose.position.z;
        const float center_dist = center_dist_from_pose > 0.0f ? center_dist_from_pose : static_cast<float>(dist(x, y, center_x, center_y));

        if (center_dist <= tolerance) {
            centered_frames++;
            cmd_vel = zero(cmd_vel);
            node->publishLocalVelocity(cmd_vel);
            if (centered_frames >= min_centered_frames) {
                RCLCPP_INFO(node->get_logger(), "CENTERED");
                centering_status = true;
                break;
            }
            rate.sleep();
            continue;
        }
        centered_frames = 0;

        const float vx = limit(step, (dy / center_y) * step);
        const float vy = limit(step, (dx / center_x) * step);
        cmd_vel.twist.linear.x = vx;
        cmd_vel.twist.linear.y = vy;
        cmd_vel.twist.linear.z = 0.0;
        node->publishLocalVelocity(cmd_vel);

        if ((now - last_log_time).seconds() >= log_interval) {
            RCLCPP_INFO(node->get_logger(), "CENTERING to x: %.1f, y: %.1f, dist: %.2f, step velocity: %.3f (vx: %.3f, vy: %.3f)", x, y, center_dist, step, vx, vy);
            last_log_time = now;
        }

        if ((now - start_time).seconds() >= max_time) {
            break;
        }

        rate.sleep();
    }

    cmd_vel = zero(cmd_vel);
    node->publishLocalVelocity(cmd_vel);
    posee = node->getCurrentLocalPose();
}

void calibrateHoverOrientation(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float max_time, float hover_pitch, float hover_roll, bool calibrate_pitch, bool calibrate_roll) {
    RCLCPP_INFO(node->get_logger(), "------------- Calibrate Hover Orientation --------------");
    stabilize(node, rate, posee, max_time);

    posee = node->getCurrentLocalPose();
    hover_pitch = calibrate_pitch ? getPitch(posee.pose.orientation) : hover_pitch;
    hover_roll = calibrate_roll ? getRoll(posee.pose.orientation) : hover_roll;

    RCLCPP_INFO(node->get_logger(), "hover_pitch = %f, hover_roll = %f", hover_pitch, hover_roll);
}

void centeringPayload(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float acc, float maxAccel, float x, float y, float min_center_time, float max_center_pitch, float max_center_roll, float hover_pitch, float hover_roll, std::string recovery_method, std::string centering_setpoint_mode) {
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
     * - status: reference to a boolean indicating whether centering is successful or not. Modified by the function (true if successful, false otherwise)
     */

    RCLCPP_INFO(node->get_logger(), "------------- Starting to Center Payload --------------");
    
    status = false;
    rclcpp::Time start_time = node->now();
    rclcpp::Time last_detected_time = node->now(); // Waktu terakhir payload terdeteksi
    const float dropAlt = 0.6;

    // const float speed_xy = 0.4;
    float roll, pitch; bool detected = false;
    geometry_msgs::msg::PoseStamped avg_payload_pose; int avg_sample = 1; int buffer_size = 4;
    std::vector<geometry_msgs::msg::PoseStamped> cache;
    geometry_msgs::msg::PoseStamped last_last_payload_pose = posee;
    geometry_msgs::msg::TwistStamped lastVel,velocity_msg;
    geometry_msgs::msg::Point error, err_i, speed_local, payload_offset;
    float payload_to_left, payload_to_front, payload_to_right, payload_to_back;
    lastVel = node->getCurrentVelocity();

    err_i.x = 0;
    err_i.y = 0;
    float speed_i = 0.0;

    payload_offset.x = -limit(node->getLastPayloadPose().pose.position.x,0.1);
    payload_offset.y = -limit(node->getLastPayloadPose().pose.position.y,0.1);

    rclcpp::Time last_center_time = rclcpp::Time(0,0);
    bool centered = false;

    if(!payloadDetected(node))
    {
        if(node->getLastNonZeroPosePayload().pose.position.x != 0.0 && node->getLastNonZeroPosePayload().pose.position.y != 0.0 && node->getLastNonZeroPosePayload().pose.position.z != 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Tadi sempet detect tapi sekarang ngilang, coba ke pose deteksi terakhir ya...");

            // Get current pose
            geometry_msgs::msg::PoseStamped last_drone_nonzero_pose = node->getDroneNonZeroPosePayload();

            // Get current payload pose in local frame
            geometry_msgs::msg::PoseStamped payload_pose = node->getLastNonZeroPosePayload();
            error = -node->getLastNonZeroPosePayload().pose.position;
            error = reflect(error);
            speed_local = point_rotation_by_quaternion(error, last_drone_nonzero_pose.pose.orientation);

            geometry_msgs::msg::PoseStamped payload_local_frame = last_drone_nonzero_pose;
            payload_local_frame.pose.position.x = last_drone_nonzero_pose.pose.position.x + speed_local.x;
            payload_local_frame.pose.position.y = last_drone_nonzero_pose.pose.position.y + speed_local.y;

            moveToPoint(node, rate, posee, payload_local_frame.pose, 0.25, acc*2.0);
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Kayaknya ini payload nya kemajuan, coba maju dikit ya...");
            
            geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
            geometry_msgs::msg::Point maju;
            float heading = getHeading(curr_pose.pose.orientation);
            maju.x = x;
            maju.y = y;
            maju = rotatePoint(maju, heading);
            
            curr_pose.pose.position.x += maju.x;
            curr_pose.pose.position.y += maju.y;
            moveToPoint(node, rate, posee, curr_pose.pose, 0.25, acc*2.0, true, false);

        }
    }

    while (rclcpp::ok()) {
        if(payloadDetected(node)){
            // Filters out old payload data to avoid drift
            if(node->getLastPayloadPose().pose.position.x == node->getCurrentPosePayload().pose.position.y && node->getLastPayloadPose().pose.position.x == node->getCurrentPosePayload().pose.position.y && node->getLastPayloadPose().pose.position.x == node->getCurrentPosePayload().pose.position.y) {
                velocity_msg = zero(velocity_msg);
                RCLCPP_INFO(node->get_logger(), "old payload position data, sending zero velocity to avoid drifting");
                node->publishLocalVelocity(velocity_msg);
                continue;
            }   

            // Get current pose
            geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
            // last_last_payload_pose=curr_pose;

            // Get current payload pose in local frame
            geometry_msgs::msg::PoseStamped payload_pose = node->getCurrentPosePayload();
            error = -node->getCurrentPosePayload().pose.position;
            error = reflect(error);
            speed_local = point_rotation_by_quaternion(error, curr_pose.pose.orientation);
            
            geometry_msgs::msg::PoseStamped payload_local_frame = curr_pose;
            payload_local_frame.pose.position.x = curr_pose.pose.position.x + speed_local.x;
            payload_local_frame.pose.position.y = curr_pose.pose.position.y + speed_local.y;
            payload_local_frame.pose.position.z = curr_pose.pose.position.z - speed_local.z;
            
            if(recovery_method == "lidar")
            {
                sensor_msgs::msg::LaserScan curr_scan = node->getCurrentLaserScan();
                payload_to_front = curr_scan.ranges[0] - error.x;
                payload_to_left  = curr_scan.ranges[(int)curr_scan.ranges.size()/4] - error.y;
                payload_to_back  = curr_scan.ranges[(int)curr_scan.ranges.size()/2] + error.x;
                payload_to_right = curr_scan.ranges[(int)curr_scan.ranges.size()*0.75] + error.y;
            }

            // ini harusnya ngelog angka yang sama terus, kalo beda beda jauh ada yang ga beres
            RCLCPP_INFO(node->get_logger(), "PAYLOAD LOCAL | x: %.2f y: %.2f z: %.2f",payload_local_frame.pose.position.x,payload_local_frame.pose.position.y,payload_local_frame.pose.position.z);

            // ======================= Filters out spikes to avoid jitters during centering (low pass filter by using moving average)
            // if(avg_sample <= 1) {avg_payload_pose = payload_local_frame; avg_sample++; cache.push_back(payload_local_frame);}
            // else if(avg_sample < buffer_size)
            // {
            //     avg_payload_pose.pose.position.x += (payload_local_frame.pose.position.x) * ((avg_sample - 1)/ avg_sample);
            //     avg_payload_pose.pose.position.y += (payload_local_frame.pose.position.y) * ((avg_sample - 1)/ avg_sample);
            //     avg_payload_pose.pose.position.z += (payload_local_frame.pose.position.z) * ((avg_sample - 1)/ avg_sample);
            //     cache.push_back(payload_local_frame);
            //     avg_sample++;
            // }
            // else
            // {
            //     cache.push_back(payload_local_frame);
            //     cache.erase(cache.begin());
            //     avg_payload_pose = centroid(cache);
            // }
            // RCLCPP_INFO(node->get_logger(), "PAYLOAD LOCAL AVG | x: %.2f y: %.2f z: %.2f", avg_payload_pose.pose.position.x, avg_payload_pose.pose.position.y, avg_payload_pose.pose.position.z);
            // const float threshold = 0.1; //trial dulu biar tau cocok nya brp
            // ========================================================================================================================================
            
            // limit speed and acceleration
            err_i.x += speed_local.x*speed_i;
            err_i.y += speed_local.y*speed_i;
            speed_local = speed_local * speed_xy;
            speed_local = speed_local + err_i;
            velocity_msg = cast(speed_local);
            velocity_msg = limitDelta(maxAccel/RATE, velocity_msg ,lastVel);
            velocity_msg = limit(0.4, velocity_msg);
            // velocity_msg.twist.linear.z = -limit(0.4, node->getCurrentRangefinder().range - dropAlt);
            
            velocity_msg.twist.linear.z = 0.0;
            
            lastVel = velocity_msg;
            
            // geometry_msgs::msg::PoseStamped pose_msg = avg_payload_pose;
            geometry_msgs::msg::PoseStamped pose_msg = payload_local_frame;
            pose_msg.pose.position.z = posee.pose.position.z;
            pose_msg.pose.orientation = posee.pose.orientation;
            
            if (centering_setpoint_mode == "velocity") {node->publishLocalVelocity(velocity_msg);}
            else if (centering_setpoint_mode == "position") {node->publishLocalPosition(pose_msg);}
            
            roll = getRoll(curr_pose.pose.orientation);
            pitch = getPitch(curr_pose.pose.orientation);
            
            // RCLCPP_INFO(node->get_logger(), "range: %f | dropAlt: %f", node->getCurrentRangefinder().range,dropAlt);
            // RCLCPP_INFO(node->get_logger(), "x: %.2f y: %.2f d: %.2f r: %.1f p: %.1f, cp: %d", payload_pose.pose.position.x,payload_pose.pose.position.y, dist(payload_pose.pose.position, -payload_offset), fabs(roll), fabs(pitch));
            RCLCPP_INFO(node->get_logger(), "Centering... dist: %f, acc: %f, roll+hvr_roll: %f, max_roll: %f, pitch+hvr_pitch: %f, max_pitch: %f", dist(error.x, error.y, 0.0, 0.0), acc, fabs(roll*180/3.14) - hover_roll, max_center_roll, fabs(pitch*180/3.14) - hover_pitch , max_center_pitch);
            
            // if (isNear(payload_local_frame.pose.position, curr_pose.pose.position, acc) && (fabs(roll*180/3.14) - hover_roll<= max_center_roll) && (fabs(pitch*180/3.14) - hover_pitch <= max_center_pitch) && !centered) {
            if (dist(error.x, error.y, 0.0, 0.0) <= acc && (fabs(roll*180/3.14) - hover_roll<= max_center_roll) && (fabs(pitch*180/3.14) - hover_pitch <= max_center_pitch) && !centered) {
                RCLCPP_INFO(node->get_logger(), "------------ Payload centered ------------");
                velocity_msg = zero(velocity_msg);
                node->publishLocalVelocity(velocity_msg);
                rclcpp::spin_some(node); rate.sleep();
                status = true; posee = curr_pose;
                last_center_time = node->now();
                centered = true;
            }
            // else if(!(isNear(payload_local_frame.pose.position, curr_pose.pose.position, acc) && (fabs(roll*180/3.14) - hover_roll <= max_center_roll) && (fabs(pitch*180/3.14) - hover_pitch <= max_center_pitch)))
            else if(!(dist(error.x, error.y, 0.0, 0.0) <= acc*1.25) && centered)
            {
                RCLCPP_INFO(node->get_logger(), "Center cancelled. dist: %f, acc: %f, roll+hvr_roll: %f, max_roll: %f, pitch+hvr_pitch: %f, max_pitch: %f", dist(error.x, error.y, 0.0, 0.0), acc, fabs(roll*180/3.14) - hover_roll, max_center_roll, fabs(pitch*180/3.14) - hover_pitch , max_center_pitch);
                centered = false;
            }
            
            last_detected_time = node->now();
            last_last_payload_pose = payload_local_frame;
            last_last_payload_pose.pose.position.z = curr_pose.pose.position.z;
            node->setLastPayloadPose(payload_pose);

            if(centered){RCLCPP_INFO(node->get_logger(), "payload centered for %.3f", (node->now() - last_center_time).seconds());}

            if(centered && node->now() - last_center_time >= rclcpp::Duration::from_seconds(min_center_time))
            {
                break;
            }
        } else {
            auto now = node->now(); detected = false;
            RCLCPP_INFO(node->get_logger(), "OII PAYLOAD MANA : %.2f", now.seconds());

            err_i.x = 0;
            err_i.y = 0;

            // --- KALO 7 detik galiat apa apa bahkan setelah recovery
            if ((now - last_detected_time).seconds() > 10 && (last_last_payload_pose.pose.position.x != 0.0 && last_last_payload_pose.pose.position.y != 0.0 && last_last_payload_pose.pose.position.z != 0.0)) {
                RCLCPP_INFO(node->get_logger(), "--- NOOO YAUDAH LANJUT AJA -----");
                status = true;
                break;
            } else if(last_last_payload_pose.pose.position.x != 0.0 && last_last_payload_pose.pose.position.y != 0.0 && last_last_payload_pose.pose.position.z != 0.0) {
                RCLCPP_INFO(node->get_logger(), "--- COBA KU SEND POSISI AWAL YAA, BISMILLAH ---");
                if(recovery_method == "local_pose")
                {
                    moveToPoint(node, rate, posee, last_last_payload_pose.pose, 0.25, acc*2.0);
                    
                    // for (int i=0; i<5; i++) {
                    //     rclcpp::spin_some(node); rate.sleep();
                    // }

                    holdPosition(node, rate, posee, 0.5);

                    if(!payloadDetected(node))
                    {
                        if(node->getLastNonZeroPosePayload().pose.position.x != 0.0 && node->getLastNonZeroPosePayload().pose.position.y != 0.0 && node->getLastNonZeroPosePayload().pose.position.z != 0.0)
                        {
                            RCLCPP_INFO(node->get_logger(), "Masi ga liat, coba ke nonzero payload pose terakhir ya...");

                            // Get current pose
                            geometry_msgs::msg::PoseStamped last_drone_nonzero_pose = node->getDroneNonZeroPosePayload();

                            // Get current payload pose in local frame
                            geometry_msgs::msg::PoseStamped payload_pose = node->getLastNonZeroPosePayload();
                            error = -node->getLastNonZeroPosePayload().pose.position;
                            error = reflect(error);
                            speed_local = point_rotation_by_quaternion(error, last_drone_nonzero_pose.pose.orientation);

                            geometry_msgs::msg::PoseStamped payload_local_frame = last_drone_nonzero_pose;
                            payload_local_frame.pose.position.x = last_drone_nonzero_pose.pose.position.x + speed_local.x;
                            payload_local_frame.pose.position.y = last_drone_nonzero_pose.pose.position.y + speed_local.y;

                            moveToPoint(node, rate, posee, payload_local_frame.pose, 0.25, acc*2.0);
                        }
                        if(!payloadDetected(node)) {
                            RCLCPP_INFO(node->get_logger(), "--- JIR MASI GA KELIATAN, KAYAKNYA GPS TOLOL, COBA MUNDUR LAHH ---");
                            geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
                            geometry_msgs::msg::Point maju;
                            float heading = getHeading(curr_pose.pose.orientation);
                            maju.x = -0.5;
                            maju.y = 0.0;
                            maju = rotatePoint(maju, heading);
                            
                            curr_pose.pose.position.x += maju.x;
                            curr_pose.pose.position.y += maju.y;
                            moveToPoint(node, rate, posee, curr_pose.pose, 0.25, acc*2.0, true, false);
                        }

                    }
                }
                else if(recovery_method == "lidar")
                {
                    RCLCPP_INFO(node->get_logger(), "Trying to recover using lidar ranges, %f, %f, %f, %f", payload_to_front, payload_to_left, payload_to_back, payload_to_right);
                    holdAtLidarRanges(node, rate, posee, (payload_to_front != std::numeric_limits<float>::infinity() && !std::isnan(payload_to_front) && payload_to_front) ? payload_to_front : -1.0, (payload_to_left != std::numeric_limits<float>::infinity() && !std::isnan(payload_to_left) && payload_to_left) ? payload_to_left : -1.0, (payload_to_back != std::numeric_limits<float>::infinity() && !std::isnan(payload_to_back) && payload_to_back) ? payload_to_back : -1.0, (payload_to_right != std::numeric_limits<float>::infinity() && !std::isnan(payload_to_right) && payload_to_right) ? payload_to_right : -1.0, false, false);
                }
            }
        }

        rclcpp::spin_some(node); rate.sleep();
    }

    posee = node->getCurrentLocalPose();
    node->resetNonzeroPayloadPose();
}

void centeringEmber(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float acc, float maxAccel, float min_center_time) {
    /**
     * Center the ember by adjusting the drone's position based on the ember's current position.
     * The drone will adjust its velocity to center the ember while maintaining a stable hover.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - speed_xy: float value representing the speed at which the drone should move in the XY plane.
     * - acc: tolerance value for determining if the ember is centered.
     * - maxAccel: float value representing the maximum acceleration allowed during centering.
     * - dropAlt: float value representing the altitude to descend to take the ember.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     * - status: reference to a boolean indicating whether centering is successful or not. Modified by the function (true if successful, false otherwise)
     */

    RCLCPP_INFO(node->get_logger(), "------------- Starting to Center Ember --------------");
    status = false;
    rclcpp::Time start_time = node->now();
    rclcpp::Time last_detected_time = node->now(); // Waktu terakhir Ember terdeteksi
    // const float dropAlt = 0.5;

    // const float speed_xy = 0.4;
    float roll, pitch; bool detected = false;
    geometry_msgs::msg::PoseStamped avg_ember_pose; int avg_sample = 1; int buffer_size = 4;
    std::vector<geometry_msgs::msg::PoseStamped> cache;
    geometry_msgs::msg::PoseStamped last_last_ember_pose = posee;
    geometry_msgs::msg::TwistStamped lastVel,velocity_msg;
    geometry_msgs::msg::Point error, err_i, speed_local, ember_offset;
    lastVel = node->getCurrentVelocity();

    err_i.x = 0;
    err_i.y = 0;
    float speed_i = 0.0;

    ember_offset.x = -limit(node->getLastEmberPose().pose.position.x,0.1);
    ember_offset.y = -limit(node->getLastEmberPose().pose.position.y,0.1);

    rclcpp::Time last_center_time = rclcpp::Time(0,0);
    bool centered = false;

    if(!emberDetected(node))
    {
        if(node->getLastNonZeroPoseEmber().pose.position.x != 0.0 && node->getLastNonZeroPoseEmber().pose.position.y != 0.0 && node->getLastNonZeroPoseEmber().pose.position.z != 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Tadi sempet detect tapi sekarang ngilang, coba ke pose deteksi terakhir ya...");

            // Get current pose
            geometry_msgs::msg::PoseStamped last_drone_nonzero_pose = node->getDroneNonZeroPoseEmber();

            // Get current ember pose in local frame
            geometry_msgs::msg::PoseStamped ember_pose = node->getLastNonZeroPoseEmber();
            error = -node->getLastNonZeroPoseEmber().pose.position;
            error = reflect(error);
            speed_local = point_rotation_by_quaternion(error, last_drone_nonzero_pose.pose.orientation);

            geometry_msgs::msg::PoseStamped ember_local_frame = last_drone_nonzero_pose;
            ember_local_frame.pose.position.x = last_drone_nonzero_pose.pose.position.x + speed_local.x;
            ember_local_frame.pose.position.y = last_drone_nonzero_pose.pose.position.y + speed_local.y;

            moveToPoint(node, rate, posee, ember_local_frame.pose, 0.25, acc*2.0);
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Kayaknya ini ember nya kemajuan, coba maju dikit ya...");
            
            geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
            geometry_msgs::msg::Point maju;
            float heading = getHeading(curr_pose.pose.orientation);
            maju.x = 1.0;
            maju = rotatePoint(maju, heading);
            
            curr_pose.pose.position.x += maju.x;
            curr_pose.pose.position.y += maju.y;
            moveToPoint(node, rate, posee, curr_pose.pose, 0.25, acc*2.0, false, true);

        }
    }

    while (rclcpp::ok()) {
        if(emberDetected(node)){

            // Filters out old ember data to avoid drift
            if(node->getLastEmberPose().pose.position.x == node->getCurrentPoseEmber().pose.position.y && node->getLastEmberPose().pose.position.x == node->getCurrentPoseEmber().pose.position.y && node->getLastEmberPose().pose.position.x == node->getCurrentPoseEmber().pose.position.y) {
                velocity_msg = zero(velocity_msg);
                RCLCPP_INFO(node->get_logger(), "old ember position data, sending zero velocity to avoid drifting");
                node->publishLocalVelocity(velocity_msg);
                continue;
            }
            
            // Get current pose
            geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
            // last_last_ember_pose=curr_pose;
            
            // Get current ember pose in local frame
            geometry_msgs::msg::PoseStamped ember_pose = node->getCurrentPoseEmber();
            error = -node->getCurrentPoseEmber().pose.position;
            error = reflect(error);
            speed_local = point_rotation_by_quaternion(error, curr_pose.pose.orientation);

            geometry_msgs::msg::PoseStamped ember_local_frame = curr_pose;
            ember_local_frame.pose.position.x = curr_pose.pose.position.x + speed_local.x;
            ember_local_frame.pose.position.y = curr_pose.pose.position.y + speed_local.y;
            ember_local_frame.pose.position.z = curr_pose.pose.position.z - speed_local.z;
            // ini harusnya ngelog angka yang sama terus, kalo beda beda jauh ada yang ga beres
            RCLCPP_INFO(node->get_logger(), "EMBER LOCAL | x: %.2f y: %.2f z: %.2f",ember_local_frame.pose.position.x,ember_local_frame.pose.position.y,ember_local_frame.pose.position.z);

            // ======================= Filters out spikes to avoid jitters during centering (low pass filter by using moving average)
            // if(avg_sample <= 1) {avg_ember_pose = ember_local_frame; avg_sample++; cache.push_back(ember_local_frame);}
            // else if(avg_sample < buffer_size)
            // {
            //     avg_ember_pose.pose.position.x += (ember_local_frame.pose.position.x) * ((avg_sample - 1)/ avg_sample);
            //     avg_ember_pose.pose.position.y += (ember_local_frame.pose.position.y) * ((avg_sample - 1)/ avg_sample);
            //     avg_ember_pose.pose.position.z += (ember_local_frame.pose.position.z) * ((avg_sample - 1)/ avg_sample);
            //     cache.push_back(ember_local_frame);
            //     avg_sample++;
            // }
            // else
            // {
            //     cache.push_back(ember_local_frame);
            //     cache.erase(cache.begin());
            //     avg_ember_pose = centroid(cache);
            // }
            // RCLCPP_INFO(node->get_logger(), "EMBER LOCAL AVG | x: %.2f y: %.2f z: %.2f", avg_ember_pose.pose.position.x, avg_ember_pose.pose.position.y, avg_ember_pose.pose.position.z);
            // const float threshold = 0.1; //trial dulu biar tau cocok nya brp
            // ========================================================================================================================================
            
            // limit speed and acceleration
            err_i.x += speed_local.x*speed_i;
            err_i.y += speed_local.y*speed_i;
            speed_local = speed_local * speed_xy;
            speed_local = speed_local + err_i;
            velocity_msg = cast(speed_local);
            velocity_msg = limitDelta(maxAccel/RATE, velocity_msg ,lastVel);
            velocity_msg = limit(0.4, velocity_msg);

            velocity_msg.twist.linear.z = 0.0;

            lastVel = velocity_msg;
            node->publishLocalVelocity(velocity_msg);

            // geometry_msgs::msg::PoseStamped pose_msg = avg_ember_pose;
            geometry_msgs::msg::PoseStamped pose_msg = ember_local_frame;
            // pose_msg.pose.position.z = curr_pose.pose.position.z;
            // node->publishLocalPosition(pose_msg);
            
            roll = getRoll(curr_pose.pose.orientation);
            pitch = getPitch(curr_pose.pose.orientation);
            
            // RCLCPP_INFO(node->get_logger(), "range: %f | dropAlt: %f", node->getCurrentRangefinder().range,dropAlt);
            // RCLCPP_INFO(node->get_logger(), "x: %.2f y: %.2f d: %.2f r: %.1f p: %.1f, cp: %d", ember_pose.pose.position.x,ember_pose.pose.position.y, dist(ember_pose.pose.position, -ember_offset), fabs(roll), fabs(pitch));
            
            if (dist(error.x, error.y, 0.0, 0.0) <= acc && !centered) {
                RCLCPP_INFO(node->get_logger(), "------------ ember centered ------------");
                velocity_msg = zero(velocity_msg);
                node->publishLocalVelocity(velocity_msg);
                status = true; posee = curr_pose;
                last_center_time = node->now();
                centered = true;
            }
            else if(!(dist(error.x, error.y, 0.0, 0.0) <= acc*1.25) && centered)
            {
                RCLCPP_INFO(node->get_logger(), "Center cancelled. dist: %f, acc: %f", dist(error.x, error.y, 0.0, 0.0), acc);
                centered = false;
            }
            
            last_detected_time = node->now();
            last_last_ember_pose = ember_local_frame;
            last_last_ember_pose.pose.position.z = curr_pose.pose.position.z;
            node->setLastEmberPose(ember_pose);

            if(centered){RCLCPP_INFO(node->get_logger(), "ember centered for %.3f", (node->now() - last_center_time).seconds());}

            if(centered && node->now() - last_center_time >= rclcpp::Duration::from_seconds(min_center_time))
            {
                break;
            }
        } else {
            auto now = node->now(); detected =false;
            RCLCPP_INFO(node->get_logger(), "OII EMBER MANA : %.2f", now.seconds());

            err_i.x = 0;
            err_i.y = 0;

            // --- KALO TBTB ILANG 7 detik
            if ((now - last_detected_time).seconds() > 10) {
                RCLCPP_INFO(node->get_logger(), "--- NOOO YAUDAH LANJUT AJA -----");
                status = true;
                break;
            } else {
                RCLCPP_INFO(node->get_logger(), "--- COBA KU SEND POSISI AWAL YAA, BISMILLAH ---");
                moveToPoint(node, rate, posee, last_last_ember_pose.pose, 0.25, acc*2.0);
                for (int i=0; i<5; i++) {
                    rclcpp::spin_some(node); rate.sleep();
                }

                if(!emberDetected(node))
                {
                    if(node->getLastNonZeroPoseEmber().pose.position.x != 0.0 && node->getLastNonZeroPoseEmber().pose.position.y != 0.0 && node->getLastNonZeroPoseEmber().pose.position.z != 0.0)
                    {
                        RCLCPP_INFO(node->get_logger(), "Masi ga liat, coba ke nonzero ember pose terakhir...");

                        // Get current pose
                        geometry_msgs::msg::PoseStamped last_drone_nonzero_pose = node->getDroneNonZeroPoseEmber();

                        // Get current ember pose in local frame
                        geometry_msgs::msg::PoseStamped ember_pose = node->getLastNonZeroPoseEmber();
                        error = -node->getLastNonZeroPoseEmber().pose.position;
                        error = reflect(error);
                        speed_local = point_rotation_by_quaternion(error, last_drone_nonzero_pose.pose.orientation);

                        geometry_msgs::msg::PoseStamped ember_local_frame = last_drone_nonzero_pose;
                        ember_local_frame.pose.position.x = last_drone_nonzero_pose.pose.position.x + speed_local.x;
                        ember_local_frame.pose.position.y = last_drone_nonzero_pose.pose.position.y + speed_local.y;

                        moveToPoint(node, rate, posee, ember_local_frame.pose, 0.25, acc*2.0);
                    }
                    if(!emberDetected(node))
                    {
                        RCLCPP_INFO(node->get_logger(), "--- JIR MASI GA KELIATAN, KAYAKNYA GPS TOLOL, COBA MUNDUR LAHH ---");
                        geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
                        geometry_msgs::msg::Point maju;
                        float heading = getHeading(curr_pose.pose.orientation);
                        maju.x = -0.5;
                        maju.y = 0.0;
                        maju = rotatePoint(maju, heading);
                        
                        curr_pose.pose.position.x += maju.x;
                        curr_pose.pose.position.y += maju.y;
                        moveToPoint(node, rate, posee, curr_pose.pose, 0.25, acc*2.0, false, true);
                    }

                }
            }
        }
        
        rclcpp::spin_some(node); rate.sleep();
    }
    
    posee = node->getCurrentLocalPose();
    node->resetNonzeroEmberPose();
}

void centeringGate(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float acc) {
    RCLCPP_INFO(node->get_logger(), "---------- Starting to center gate ----------");
    
    while(rclcpp::ok()) {
        sensor_msgs::msg::LaserScan curr_scan = node->getCurrentLaserScan();
        geometry_msgs::msg::TwistStamped cmd_vel;
        geometry_msgs::msg::Point vel_pose;
        bool one_centroid_only = false;
        
        // 1. Get points from polar coordinate (laserscan data) to cartesian coordinate
        int quarter_size = curr_scan.ranges.size()/4;
        int left_count = 0; int right_count = 0;
        std::vector<float> x_points;
        std::vector<float> y_points;
        RCLCPP_INFO(node->get_logger(), "LEFT: ");
        for(int i = 0; i <= quarter_size; i++) {
            float angle = M_PI/(2*quarter_size) * i;
            if(curr_scan.ranges[i] == std::numeric_limits<float>::infinity() || std::isnan(curr_scan.ranges[i]) || curr_scan.ranges[i] > 2.82843 || curr_scan.ranges[i]*cos(angle) < 0.4 || curr_scan.ranges[i] < 0.4)
                continue;
            x_points.push_back(curr_scan.ranges[i]*cos(angle));
            y_points.push_back(curr_scan.ranges[i]*sin(angle));
            RCLCPP_INFO(node->get_logger(), "   - x: %f, y: %f, angle: %f", curr_scan.ranges[i]*cos(angle), curr_scan.ranges[i]*sin(angle), angle);
            left_count++;
        }
        RCLCPP_INFO(node->get_logger(), "RIGHT: ");
        for(int i = 0; i <= quarter_size; i++) {
            float angle = M_PI*1.5 + (M_PI/(2*quarter_size) * i);
            if(curr_scan.ranges[3*quarter_size + i] == std::numeric_limits<float>::infinity() || std::isnan(curr_scan.ranges[3*quarter_size + i]) || curr_scan.ranges[3*quarter_size + i] > 2.0 || curr_scan.ranges[3*quarter_size + i]*cos(angle) < 0.4 || curr_scan.ranges[3*quarter_size + i] < 0.4)
                continue;
            x_points.push_back(curr_scan.ranges[3*quarter_size + i]*cos(angle));
            y_points.push_back(curr_scan.ranges[3*quarter_size + i]*sin(angle));
            RCLCPP_INFO(node->get_logger(), "   - x: %f, y: %f, angle: %f", curr_scan.ranges[3*quarter_size + i]*cos(angle), curr_scan.ranges[3*quarter_size + i]*sin(angle), angle);
            right_count++;
        }
        if(x_points.size() < 2) {
            RCLCPP_ERROR(node->get_logger(), "GOT NO POINTS FROM LIDAR DATA");
            cmd_vel = zero(cmd_vel);
            node->publishLocalVelocity(cmd_vel);
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }

        // 2. Do clustering with two centroid
        int iteration = 50;
        // float centroid1_x = 1.0, centroid1_y = 0.75, centroid2_x = 1.0, centroid2_y = -0.75;
        float centroid1_x = *std::min_element(x_points.begin(), x_points.end()), centroid1_y = *std::max_element(y_points.begin(), y_points.end()), centroid2_x = *std::min_element(x_points.begin(), x_points.end()), centroid2_y = *std::min_element(y_points.begin(), y_points.end());
        int centroid1_member_size, centroid2_member_size;
        float x1_min = -1.0, x2_min = -1.0;
        geometry_msgs::msg::Point min_x_centroid1, min_x_centroid2;

        for(int i = 0; i < iteration; i++) {
            min_x_centroid1.x = 99999;
            min_x_centroid2.x = 99999;
            
            float temp_centroid1_x = 0.0, temp_centroid1_y = 0.0, temp_centroid2_x = 0.0, temp_centroid2_y = 0.0;
            centroid1_member_size = 0;
            centroid2_member_size= 0;
            for(int j = 0; j < x_points.size(); j++) {
                float dist_to_1 = dist(centroid1_x, centroid1_y, x_points[j], y_points[j]);
                float dist_to_2 = dist(centroid2_x, centroid2_y, x_points[j], y_points[j]);

                if(dist_to_1 < dist_to_2){
                    temp_centroid1_x += x_points[j];
                    temp_centroid1_y += y_points[j];
                    centroid1_member_size++;
                    if(x_points[j] < min_x_centroid1.x)
                    {
                        min_x_centroid1.x = x_points[j];
                        min_x_centroid1.y = y_points[j];
                    }
                }
                else {
                    temp_centroid2_x += x_points[j];
                    temp_centroid2_y += y_points[j];
                    centroid2_member_size++;
                    if(x_points[j] < min_x_centroid2.x)
                    {
                        min_x_centroid2.x = x_points[j];
                        min_x_centroid2.y = y_points[j];
                    }
                }
            }

            if(!centroid1_member_size || !centroid2_member_size){
                RCLCPP_ERROR(node->get_logger(), "Centroid1_member_size: %d, Centroid2_member_size: %d", centroid1_member_size, centroid2_member_size);
                break;
            }

            centroid1_x = temp_centroid1_x / centroid1_member_size;
            centroid1_y = temp_centroid1_y / centroid1_member_size;
            centroid2_x = temp_centroid2_x / centroid2_member_size;
            centroid2_y = temp_centroid2_y / centroid2_member_size;
        }

        // 3. Fix yaw if necessary
        // float error_heading = atan2(min_x_centroid2.x - min_x_centroid1.x, 1.5);
        // if(error_heading >= 5.0*DEG2RAD || error_heading <= -5.0*DEG2RAD)
        // {
        //     RCLCPP_INFO(node->get_logger(), "fixing yaw, error: %f", error_heading*RAD2DEG);
        //     geometry_msgs::msg::PoseStamped last_pose = node->getCurrentLocalPose();
        //     setHeading(last_pose.pose.orientation, getHeading(last_pose.pose.orientation) + error_heading);
        //     for (int i = 0; i < 20; i++)
        //     {
        //         node->publishLocalPosition(last_pose);
        //     }
        //     holdPosition(node, rate, posee, 0.3);
        //     continue;
        // }

        // 4. Move to the center of the clusters
        if(centroid1_member_size && centroid2_member_size)
        {
            if(min_x_centroid1.y - min_x_centroid2.y < 0.75) {
                one_centroid_only = true;
                if(min_x_centroid1.y < 0.0) {
                    vel_pose.y = 0.75 + min_x_centroid2.y;
                }
                else if(min_x_centroid2.y > 0.0) {
                    vel_pose.y = min_x_centroid1.y - 0.75;
                }
            }
            else {
                vel_pose.y = min_x_centroid1.y + min_x_centroid2.y;
            }
        }
        else if(centroid1_member_size && !centroid2_member_size)
        {
            one_centroid_only = true;
            vel_pose.y = min_x_centroid1.y - 0.75;
        }
        else if(!centroid1_member_size && centroid2_member_size)
        {
            one_centroid_only = true;
            vel_pose.y = 0.75 + min_x_centroid2.y;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "No Points..");
            vel_pose.y = 0.0;
        }

        if(std::min(min_x_centroid1.x, min_x_centroid2.x) > 1.25)
        {
            vel_pose.x = std::min(min_x_centroid1.x, min_x_centroid2.x) - 1.25;
        }
        
        RCLCPP_INFO(node->get_logger(), "cen1x: %f, cen1y: %f, cen2x: %f, cen2y: %f, velx: %f, vely: %f, mem1: %d, mem2: %d", min_x_centroid1.x, min_x_centroid1.y, min_x_centroid2.x, min_x_centroid2.y, vel_pose.x, vel_pose.y, centroid1_member_size, centroid2_member_size);
        
        
        if(vel_pose.y && std::abs(vel_pose.y) <= acc && (min_x_centroid1.y - min_x_centroid2.y >= 1.25 || one_centroid_only))
        {
            RCLCPP_INFO(node->get_logger(), "---------- SUDAH CENTER ----------");
            cmd_vel = zero(cmd_vel);
            node->publishLocalVelocity(cmd_vel);
            break;
        }
        // vel_pose.y = limit(0.1, vel_pose.y);
        if(vel_pose.y >= 0.1){vel_pose.y = 0.1;}
        else if(vel_pose.y <= -0.1){vel_pose.y = -0.1;}
        float drone_heading = getHeading(node->getCurrentLocalPose().pose.orientation);
        vel_pose = rotatePoint(vel_pose, drone_heading);
        cmd_vel = cast(vel_pose);
        node->publishLocalVelocity(cmd_vel);

        rclcpp::spin_some(node); rate.sleep();
    }

    posee = node->getCurrentLocalPose();
}

void centeringBunder(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float speed_xy, bool &status, float acc, float maxAccel, float min_center_time) {
    /**
     * Center the ember by adjusting the drone's position based on the ember's current position.
     * The drone will adjust its velocity to center the ember while maintaining a stable hover.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - speed_xy: float value representing the speed at which the drone should move in the XY plane.
     * - acc: tolerance value for determining if the ember is centered.
     * - maxAccel: float value representing the maximum acceleration allowed during centering.
     * - dropAlt: float value representing the altitude to descend to take the ember.
     * 
     * returns:
     * - posee: reference to where the last drone position when this function is called, it will be modified to the last drone position when this function returns.
     * - status: reference to a boolean indicating whether centering is successful or not. Modified by the function (true if successful, false otherwise)
     */

    RCLCPP_INFO(node->get_logger(), "------------- Starting to Center Bunder --------------");
    status = false;
    rclcpp::Time start_time = node->now();
    rclcpp::Time last_detected_time = node->now(); // Waktu terakhir Bunder terdeteksi
    const float dropAlt = 0.85;

    // const float speed_xy = 0.4;
    float roll, pitch; bool detected = false;
    geometry_msgs::msg::PoseStamped avg_bunder_pose; int avg_sample = 1; int buffer_size = 4;
    std::vector<geometry_msgs::msg::PoseStamped> cache;
    geometry_msgs::msg::PoseStamped last_last_bunder_pose = posee;
    geometry_msgs::msg::TwistStamped lastVel,velocity_msg;
    geometry_msgs::msg::Point error, err_i, speed_local, bunder_offset;
    lastVel = node->getCurrentVelocity();

    err_i.x = 0;
    err_i.y = 0;
    float speed_i = 0.0;
    
    bunder_offset.x = -limit(node->getLastBunderPose().pose.position.x,0.1);
    bunder_offset.y = -limit(node->getLastBunderPose().pose.position.y,0.1);

    rclcpp::Time last_center_time = rclcpp::Time(0,0);
    bool centered = false;

    if(!bunderDetected(node))
    {
        if(node->getLastNonZeroPoseBunder().pose.position.x != 0.0 && node->getLastNonZeroPoseBunder().pose.position.y != 0.0 && node->getLastNonZeroPoseBunder().pose.position.z != 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Tadi sempet detect tapi sekarang ngilang, coba ke pose deteksi terakhir ya...");

            // Get current pose
            geometry_msgs::msg::PoseStamped last_drone_nonzero_pose = node->getDroneNonZeroPoseBunder();

            // Get current bunder pose in local frame
            geometry_msgs::msg::PoseStamped bunder_pose = node->getLastNonZeroPoseBunder();
            error = -node->getLastNonZeroPoseBunder().pose.position;
            error = reflect(error);
            speed_local = point_rotation_by_quaternion(error, last_drone_nonzero_pose.pose.orientation);

            geometry_msgs::msg::PoseStamped bunder_local_frame = last_drone_nonzero_pose;
            bunder_local_frame.pose.position.x = last_drone_nonzero_pose.pose.position.x + speed_local.x;
            bunder_local_frame.pose.position.y = last_drone_nonzero_pose.pose.position.y + speed_local.y;

            moveToPoint(node, rate, posee, bunder_local_frame.pose, 0.5, 0.075);
        }
        // else
        // {
        //     RCLCPP_INFO(node->get_logger(), "Kayaknya ini bunder nya kemajuan, coba maju dikit ya...");
            
        //     geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
        //     geometry_msgs::msg::Point maju;
        //     float heading = getHeading(curr_pose.pose.orientation);
        //     maju.x = 1.0;
        //     maju = rotatePoint(maju, heading);
            
        //     curr_pose.pose.position.x += maju.x;
        //     curr_pose.pose.position.y += maju.y;
        //     moveToPoint(node, rate, posee, curr_pose.pose, 0.1, 0.075, false, true);

        // }
    }

    while (rclcpp::ok()) {
        if(bunderDetected(node)){

            // Filters out old bunder data to avoid drift
            if(node->getLastBunderPose().pose.position.x == node->getCurrentPoseBunder().pose.position.y && node->getLastBunderPose().pose.position.x == node->getCurrentPoseBunder().pose.position.y && node->getLastBunderPose().pose.position.x == node->getCurrentPoseBunder().pose.position.y) {
                velocity_msg = zero(velocity_msg);
                RCLCPP_INFO(node->get_logger(), "old bunder position data, sending zero velocity to avoid drifting");
                node->publishLocalVelocity(velocity_msg);
                continue;
            }
            
            // Get current pose
            geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
            // last_last_bunder_pose=curr_pose;
            
            // Get current bunder pose in local frame
            geometry_msgs::msg::PoseStamped bunder_pose = node->getCurrentPoseBunder();
            error = -node->getCurrentPoseBunder().pose.position;
            error = reflect(error);
            speed_local = point_rotation_by_quaternion(error, curr_pose.pose.orientation);

            geometry_msgs::msg::PoseStamped bunder_local_frame = curr_pose;
            bunder_local_frame.pose.position.x = curr_pose.pose.position.x + speed_local.x;
            bunder_local_frame.pose.position.y = curr_pose.pose.position.y + speed_local.y;
            bunder_local_frame.pose.position.z = curr_pose.pose.position.z - speed_local.z;
            // ini harusnya ngelog angka yang sama terus, kalo beda beda jauh ada yang ga beres
            RCLCPP_INFO(node->get_logger(), "BUNDER LOCAL | x: %.2f y: %.2f z: %.2f",bunder_local_frame.pose.position.x,bunder_local_frame.pose.position.y,bunder_local_frame.pose.position.z);

            // ======================= Filters out spikes to avoid jitters during centering (low pass filter by using moving average)
            // if(avg_sample <= 1) {avg_bunder_pose = bunder_local_frame; avg_sample++; cache.push_back(bunder_local_frame);}
            // else if(avg_sample < buffer_size)
            // {
            //     avg_bunder_pose.pose.position.x += (bunder_local_frame.pose.position.x) * ((avg_sample - 1)/ avg_sample);
            //     avg_bunder_pose.pose.position.y += (bunder_local_frame.pose.position.y) * ((avg_sample - 1)/ avg_sample);
            //     avg_bunder_pose.pose.position.z += (bunder_local_frame.pose.position.z) * ((avg_sample - 1)/ avg_sample);
            //     cache.push_back(bunder_local_frame);
            //     avg_sample++;
            // }
            // else
            // {
            //     cache.push_back(bunder_local_frame);
            //     cache.erase(cache.begin());
            //     avg_bunder_pose = centroid(cache);
            // }
            // RCLCPP_INFO(node->get_logger(), "BUNDER LOCAL AVG | x: %.2f y: %.2f z: %.2f", avg_bunder_pose.pose.position.x, avg_bunder_pose.pose.position.y, avg_bunder_pose.pose.position.z);
            // const float threshold = 0.1; //trial dulu biar tau cocok nya brp
            // ========================================================================================================================================
            
            // limit speed and acceleration
            err_i.x += speed_local.x*speed_i;
            err_i.y += speed_local.y*speed_i;
            speed_local = speed_local * speed_xy;
            speed_local = speed_local + err_i;
            velocity_msg = cast(speed_local);
            velocity_msg = limitDelta(maxAccel/RATE, velocity_msg ,lastVel);
            velocity_msg = limit(0.4, velocity_msg);
            velocity_msg.twist.linear.z = -limit(0.4, node->getCurrentRangefinder().range - dropAlt);

            // velocity_msg.twist.linear.z = 0.0;

            lastVel = velocity_msg;
            node->publishLocalVelocity(velocity_msg);

            // geometry_msgs::msg::PoseStamped pose_msg = avg_bunder_pose;
            geometry_msgs::msg::PoseStamped pose_msg = bunder_local_frame;
            // pose_msg.pose.position.z = curr_pose.pose.position.z;
            // node->publishLocalPosition(pose_msg);
            
            roll = getRoll(curr_pose.pose.orientation);
            pitch = getPitch(curr_pose.pose.orientation);
            
            // RCLCPP_INFO(node->get_logger(), "range: %f | dropAlt: %f", node->getCurrentRangefinder().range,dropAlt);
            // RCLCPP_INFO(node->get_logger(), "x: %.2f y: %.2f d: %.2f r: %.1f p: %.1f, cp: %d", bunder_pose.pose.position.x,bunder_pose.pose.position.y, dist(bunder_pose.pose.position, -bunder_offset), fabs(roll), fabs(pitch));
            
            if (isNear(bunder_local_frame.pose.position, curr_pose.pose.position, acc) && !centered && node->getCurrentRangefinder().range <= dropAlt) {
                RCLCPP_INFO(node->get_logger(), "------------ bunder centered ------------");
                velocity_msg = zero(velocity_msg);
                node->publishLocalVelocity(velocity_msg);
                status = true; posee = curr_pose;
                last_center_time = node->now();
                centered = true;
            }
            else if(!(isNear(bunder_local_frame.pose.position, curr_pose.pose.position, acc)))
            {
                centered = false;
            }
            
            last_detected_time = node->now();
            last_last_bunder_pose = bunder_local_frame;
            last_last_bunder_pose.pose.position.z = curr_pose.pose.position.z;
            node->setLastBunderPose(bunder_pose);

            if(centered){RCLCPP_INFO(node->get_logger(), "bunder centered for %.3f", (node->now() - last_center_time).seconds());}

            if(centered && node->now() - last_center_time >= rclcpp::Duration::from_seconds(min_center_time))
            {
                break;
            }
        } else {
            auto now = node->now(); detected =false;
            RCLCPP_INFO(node->get_logger(), "OII BUNDER MANA : %.2f", now.seconds());

            err_i.x = 0;
            err_i.y = 0;

            // --- KALO TBTB ILANG 7 detik
            if ((now - last_detected_time).seconds() > 5) {
                RCLCPP_INFO(node->get_logger(), "--- NOOO YAUDAH LANJUT AJA -----");
                status = true;
                break;
            } else {
                RCLCPP_INFO(node->get_logger(), "--- COBA KU SEND POSISI AWAL YAA, BISMILLAH ---");
                moveToPoint(node, rate, posee, last_last_bunder_pose.pose, 0.5, 0.075);
            }
        }

        rclcpp::spin_some(node); rate.sleep();
    }

    posee = node->getCurrentLocalPose();
    node->resetNonzeroBunderPose();
}

void moveToPoint(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float x, float y, float z, float angle, float speed, float tolerance) {
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
    float last_x, last_y, distance, progress=0.0; int i =0;
    geometry_msgs::msg::PoseStamped cmd_pose = node->getCurrentLocalPose();
    last_x = cmd_pose.pose.position.x;
    last_y = cmd_pose.pose.position.y;
    distance = dist(cmd_pose.pose.position,x,y);
    geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();
    while(rclcpp::ok() && !isNear(curr_pose.pose.position, x, y, tolerance)){
        RCLCPP_INFO(node->get_logger(), "dist: %f goto progress: %f",distance,progress);
        cmd_pose.pose.position.x = last_x+progress*(x-last_x);
        cmd_pose.pose.position.y = last_y+progress*(y-last_y);
        cmd_pose.pose.position.z = z + (node->getCurrentLocalPose().pose.position.z - node->getCurrentRangefinder().range);
        progress = (progress<1)? progress+1/((distance/speed)*20.0): 1.0;
        node->publishLocalPosition(cmd_pose);
        rclcpp::spin_some(node);
        rate.sleep();
        curr_pose = node->getCurrentLocalPose();
    }
    
    posee = node->getCurrentLocalPose();
}

void moveToPoint(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, geometry_msgs::msg::Pose target, float speed, float tolerance, bool allow_centering_payload, bool allow_centering_ember, bool disable_z_lock) {
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
     * returns:moveToPoint
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */
    // float last_x, last_y, distance, progress=0.0; int i =0;
    geometry_msgs::msg::PoseStamped cmd_pose = node->getCurrentLocalPose();
    cmd_pose.pose = target;
    
    // last_x = cmd_pose.pose.position.x;
    // last_y = cmd_pose.pose.position.y;
    // distance = dist(cmd_pose.pose.position, target.position);
    geometry_msgs::msg::PoseStamped curr_pose = node->getCurrentLocalPose();

    float distance_to_target = dist(posee.pose.position, target.position);
    float vel_z = (target.position.z - node->getCurrentRangefinder().range) / dist(curr_pose.pose.position, target.position);
    bool alt_reached = false;

    while(rclcpp::ok() && !isNear(curr_pose.pose.position, target.position, tolerance)){
        float distance = dist(curr_pose.pose.position, target.position);
        RCLCPP_INFO(node->get_logger(), "moving..., dist: %f",distance);
        curr_pose = node->getCurrentLocalPose();

        geometry_msgs::msg::TwistStamped vel;
        vel.twist.linear.x = (target.position.x - curr_pose.pose.position.x);
        vel.twist.linear.y = (target.position.y - curr_pose.pose.position.y);
        
        vel = limit(speed, vel);
        vel.twist.linear.z = disable_z_lock ? (std::abs(target.position.z - node->getCurrentRangefinder().range) > 0.15 && alt_reached ? vel_z : 0.8*(target.position.z - node->getCurrentRangefinder().range)) : 0.0;
        alt_reached = std::abs(target.position.z - node->getCurrentRangefinder().range) > 0.15 && !alt_reached;
        node->publishLocalVelocity(vel);
        
        rclcpp::spin_some(node);
        rate.sleep();
        
        if(payloadDetected(node) && allow_centering_payload)
        {
            RCLCPP_INFO(node->get_logger(), "Payload Spotted");
            geometry_msgs::msg::TwistStamped vel;
            vel = zero(vel);
            node->publishLocalVelocity(vel);
            rclcpp::spin_some(node); rate.sleep();
            break;
        }
        if(emberDetected(node) && allow_centering_ember)
        {
            RCLCPP_INFO(node->get_logger(), "Ember Spotted");
            geometry_msgs::msg::TwistStamped vel;
            vel = zero(vel);
            node->publishLocalVelocity(vel);
            rclcpp::spin_some(node); rate.sleep();
            break;
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "NYAMPEEEE");
    geometry_msgs::msg::TwistStamped vel;
    vel = zero(vel);
    node->publishLocalVelocity(vel);
    rclcpp::spin_some(node); rate.sleep();
    posee = node->getCurrentLocalPose();
}

void landingFaux(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee){
    RCLCPP_INFO(node->get_logger(), "===================== Turun ===================");
    geometry_msgs::msg::TwistStamped vel;
    bool was_descending = false, trying_descending = false;

    // float heading;
    // geometry_msgs::msg::PoseStamped last_last_payload_pose = posee;
    // geometry_msgs::msg::TwistStamped lastVel, vel;
    // geometry_msgs::msg::Point error, err_i, speed_local,payload_offset;

    // lastVel = node->getCurrentVelocity();

    // err_i.x = 0;
    // err_i.y = 0;
    // float speed_i = 0.0;
    // float test = 0;

    // payload_offset.x = -limit(node->getLastPayloadPose().pose.position.x,0.1);
    // payload_offset.y = -limit(node->getLastPayloadPose().pose.position.y,0.1);

    
    while (rclcpp::ok()){
        // heading = getHeading(posee.pose.orientation);
        // error = -node->getCurrentPosePayload().pose.position-payload_offset;
        // error = reflect(error);
        // speed_local = rotatePoint(error,heading);

        // err_i.x += speed_local.x*speed_i;
        // err_i.y += speed_local.y*speed_i;

        // speed_local = speed_local * 0.275;
        // speed_local = speed_local + err_i;

        // vel = cast(speed_local);
        // vel = limitDelta(0.3/RATE, vel ,lastVel);
        // vel = limit(0.4, vel);

        // vel.twist.linear.z = -0.625;
        // vel.twist.linear.x = posee.pose.position.x - node->getCurrentLocalPose().pose.position.x;
        // vel.twist.linear.y = posee.pose.position.y - node->getCurrentLocalPose().pose.position.y;
        // vel.twist.linear.z = -0.725;
        vel.twist.linear.x = 0.0;
        vel.twist.linear.y = 0.0;
        vel.twist.linear.z = -0.69;

        node->publishLocalVelocity(vel);
        if (node->getCurrentVelocity().twist.linear.z < -0.05 && !trying_descending)  { RCLCPP_INFO(node->get_logger(), "Descend started");trying_descending = true;}
        if (node->getCurrentVelocity().twist.linear.z < -0.35)                        { RCLCPP_INFO(node->get_logger(), "Descending...")  ;was_descending = true;}
        if (was_descending && payloadDetected(node))                                  { node->setLastPayloadPose(node->getCurrentPosePayload());}
        if (abs(node->getCurrentVelocity().twist.linear.z) < 0.055 && was_descending) { RCLCPP_INFO(node->get_logger(), "Landed"); break;}
        rclcpp::spin_some(node);rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Latest offset: %f, %f",node->getLastPayloadPose().pose.position.x,node->getLastPayloadPose().pose.position.y);
    posee = node->getCurrentLocalPose();
}

void landingFauxPose(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee){
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
    RCLCPP_INFO(node->get_logger(), "===================== Turun ====================");

    geometry_msgs::msg::PoseStamped pose = posee;

    float descend_speed = 0.4; 
    bool was_descending=false, trying_descending = false;
    
    while(rclcpp::ok()){
        pose.pose.position.x = posee.pose.position.x;
        pose.pose.position.y = posee.pose.position.y;
        pose.pose.position.z -= descend_speed/RATE;
        node->publishLocalPosition(pose);

        if (node->getCurrentVelocity().twist.linear.z<-0.05 && !trying_descending){  RCLCPP_INFO(node->get_logger(),"Descending..."); trying_descending = true; }
        if (node->getCurrentVelocity().twist.linear.z<-0.3){ was_descending =true;}
        if (abs(node->getCurrentVelocity().twist.linear.z)<0.05 && was_descending){RCLCPP_INFO(node->get_logger(), "Landed"); posee = node->getCurrentLocalPose(); break;}

        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void holdPosition(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped& posee, float duration){
    /**
     * Hold current position for a specified duration.
     * For PX4 OFFBOARD mode: MUST continuously stream setpoints at high rate
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate.
     * - duration: float value representing the duration in seconds to hold the position.
     */
    RCLCPP_INFO(node->get_logger(), "Holding position for %.1f seconds...", duration);
    
    rclcpp::Time start_time = node->now();
    rclcpp::Rate fast_rate(20.0); // 20Hz for PX4 OFFBOARD
    
    // Get current position to hold
    geometry_msgs::msg::PoseStamped hold_pose = node->getCurrentLocalPose();
    
    while (rclcpp::ok()) {
        rclcpp::Time current_time = node->now();
        double elapsed_time = (current_time - start_time).seconds();
        
        // PX4 CRITICAL: Must stream setpoint continuously
        hold_pose.header.stamp = node->now();
        node->publishLocalPosition(hold_pose);
        
        if (fmod(elapsed_time, 1.0) < 0.05) { // Log every ~1 second
            RCLCPP_INFO(node->get_logger(), "Holding... %.1f/%.1f seconds", elapsed_time, duration);
        }
        
        if (elapsed_time > duration){break;}

        rclcpp::spin_some(node); 
        fast_rate.sleep(); // Use fast rate for continuous streaming
    }
    
    RCLCPP_INFO(node->get_logger(), "Hold position complete");
    posee = node->getCurrentLocalPose();
}

void waitForPosition(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, mavros_msgs::msg::GlobalPositionTarget raw, double tolerance, bool allow_centering, float center_dist) {
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
    sensor_msgs::msg::NavSatFix init_global = node->getCurrentGPSPosition();

    while (rclcpp::ok()) {
        double dlat = raw.latitude - node->getCurrentGPSPosition().latitude;
        double dlon = raw.longitude - node->getCurrentGPSPosition().longitude;
        double dalt = raw.altitude - node->getCurrentGPSPosition().altitude;

        double distance = std::sqrt(dlat * dlat + dlon * dlon);
        double dist_m = latLonToMeter(init_global.latitude, init_global.longitude, node->getCurrentGPSPosition().latitude, node->getCurrentGPSPosition().longitude);

        RCLCPP_INFO(node->get_logger(), "%d - CURRR DISTANCEE: %f, meter: %.2f, payload: %.2f", distance, dist_m, node->getCurrentPosePayload().pose.position.x);

        if (distance < tolerance) {
            RCLCPP_INFO(node->get_logger(), "Reached target positionnnn");
            break;
        }

        if(payloadDetected(node) && allow_centering){
            RCLCPP_INFO(node->get_logger(), "Payload spotted");
            // distancee_to_payload = sqrt((pose.pose.position.x - init_pose.pose.position.x) * (pose.pose.position.x - init_pose.pose.position.x) + (pose.pose.position.y - init_pose.pose.position.y) * (pose.pose.position.y - init_pose.pose.position.y));
            break;
        }

        node->publishSetpointRawGlobal(raw);
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void sendGlobalRaw(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float latt, float lonn, float altt, bool allow_centering, float center_dist, float tolerance) {
    /**
     * Send a raw global position target to the drone and wait until the drone reaches the target position.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - latt: float value representing the target latitude in degrees.
     * - lonn: float value representing the target longitude in degrees.
     * - altt: float value representing the target altitude in meters.
     * - allow_centering: boolean value indicating whether to allow centering the drone based on the payload position.
     * - center_dist: float value representing the distance in meters to center the drone based on the payload position.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */
    mavros_msgs::msg::GlobalPositionTarget raw;
    raw.header.stamp = node->now();
    raw.header.frame_id = "map";
    raw.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    raw.type_mask = 
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW_RATE;

    raw.latitude = latt;
    raw.longitude = lonn;
    raw.altitude = altt;

    RCLCPP_INFO(node->get_logger(), "Sending target: [%.8f, %.8f, %.2f]", latt, lonn, altt);

    for (int i = 0; i < 20; i++) {
        node->publishSetpointRawGlobal(raw);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    waitForPosition(node, rate, raw, tolerance, allow_centering, center_dist);
    // waitForPosition(node, rate, latt, lonn, altt, );

    posee = node->getCurrentLocalPose();
}

void sendGlobalRawAsync(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float latt, float lonn, float altt, bool allow_centering, float center_dist) {
    /**
     * Send a raw global position target to the drone and wait until the drone reaches the target position.
     * 
     * parameters:
     * - node: shared pointer to the DroneController node instance.
     * - rate: reference to the rclcpp::Rate object for controlling the loop rate
     * - latt: float value representing the target latitude in degrees.
     * - lonn: float value representing the target longitude in degrees.
     * - altt: float value representing the target altitude in meters.
     * - allow_centering: boolean value indicating whether to allow centering the drone based on the payload position.
     * - center_dist: float value representing the distance in meters to center the drone based on the payload position.
     * 
     * returns:
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     */
    mavros_msgs::msg::GlobalPositionTarget raw;
    raw.header.stamp = node->now();
    raw.header.frame_id = "map";
    raw.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    raw.type_mask = 
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW_RATE;

    raw.latitude = latt;
    raw.longitude = lonn;
    raw.altitude = altt;

    RCLCPP_INFO(node->get_logger(), "Sending target: [%.8f, %.8f, %.2f]", latt, lonn, altt);

    for (int i = 0; i < 1; i++) {
        node->publishSetpointRawGlobal(raw);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // waitForPosition(node, rate, raw, 0.0000025, allow_centering, center_dist);
    // waitForPosition(node, rate, latt, lonn, altt, );

    posee = node->getCurrentLocalPose();
}

void sendGlobalRawFromXYZ(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, LatLonAlt zero, double zero_hdg, float x, float y, float rel_alt, bool allow_centering, float center_dist) {
    mavros_msgs::msg::GlobalPositionTarget raw = create_globalpos_from_xyz(x, y, rel_alt, zero, zero_hdg);
    raw.header.stamp = node->now();
    RCLCPP_INFO(node->get_logger(), "Sending target: [%.8f, %.8f, %.2f]", raw.latitude, raw.longitude, raw.altitude);

    for (int i = 0; i < 20; i++) {
        node->publishSetpointRawGlobal(raw);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // waitForPosition(node, rate, raw.latitude, raw.longitude, raw.altitude);

    posee = node->getCurrentLocalPose();
}

void correctAltitude(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, double height, float &corrected_rel_alt, double max_speed) {
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
     * - posee: reference to the geometry_msgs::msg::PoseStamped object representing the current pose of the drone.
     * - The correct relative altitude after adjusting the drone's position. ready to be used for sendGlobalRaw.
     */
    RCLCPP_INFO(node->get_logger(), "fixing altitude..");
    geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
    rclcpp::Time last_correct_time = rclcpp::Time(0,0);
    bool hit = false;

    const float MAX_SPEED = max_speed;

    const float P = 1.0;
    const float I = 0.0;
    const float D = 0.0;

    float total_error = 0;
    float last_error = 0;
    
    while
    (   
        !hit ||
        (rclcpp::ok() && abs(node->getCurrentRangefinder().range-height)>0.05) ||
        (hit && node->now() - last_correct_time <= rclcpp::Duration::from_seconds(2.0))
    )   
    {
        total_error += height - node->getCurrentRangefinder().range;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = limit(MAX_SPEED, P*(height - node->getCurrentRangefinder().range) + I*(total_error) + D*(last_error - (height - node->getCurrentRangefinder().range)));

        RCLCPP_INFO(node->get_logger(), "height: %f current_alt.range: %f",height,node->getCurrentRangefinder().range);
        last_error = (height - node->getCurrentRangefinder().range);
        node->publishLocalVelocityUnstamped(cmd_vel);
        rclcpp::spin_some(node);
        rate.sleep();

        if(!(abs(node->getCurrentRangefinder().range-height)>0.05) && !hit)
        {
            last_correct_time = node->now();
            hit = true;
            RCLCPP_INFO(node->get_logger(), "HIT!, last_correct_time: %f",last_correct_time.seconds());
        }
        else if((abs(node->getCurrentRangefinder().range-height)>0.05))
        {
            RCLCPP_INFO(node->get_logger(), "left the tolerance zone, resetting");
            hit = false;
        }

    }
    corrected_rel_alt = node->getCurrentRelAlt().data;
    posee = node->getCurrentLocalPose();
}

void controlServo(const std::shared_ptr<DroneController>&node, int channel, int pwm) {
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    // MAV_CMD_DO_SET_SERVO == 183
    request->command = 183; request->param1 = channel; request->param2 = pwm;
    request->param3 = 0; request->param4 = 0; request->param5 = 0; request->param6 = 0; request->param7 = 0;

    auto future_result = node->servo_(request);

    if (hasReplied(node,future_result)) { auto result = future_result.get();
        if (result->success) { RCLCPP_INFO(node->get_logger(), "Channel: %d servo signal sent", channel);} 
        else                 { RCLCPP_ERROR(node->get_logger(), "Servo signal failed.");}
    } else {RCLCPP_ERROR(node->get_logger(), "Failed to receive response");}
}

void controlServoRepeated(const std::shared_ptr<DroneController>& node, int channel, int pwm, int repeat) {
    for (int i = 0; i < repeat; ++i) {
        controlServo(node, channel, pwm);
    }
}

void pushMission(const std::shared_ptr<DroneController>&node, std::vector<mavros_msgs::msg::Waypoint> waypoints) {
    waypoints.insert(waypoints.begin() + 0, waypoints[0]); // I dont know why but the first waypoint is always skipped when pushing mission, so I duplicate it
    RCLCPP_INFO(node->get_logger(), "Pushing mission of size %d", waypoints.size());
    auto request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    request->start_index = 0;
    request->waypoints = waypoints;

    auto future_result = node->pushMission_(request);

    if (hasReplied(node,future_result)) { auto result = future_result.get();
        if (result->success) { RCLCPP_INFO(node->get_logger(), "Mission pushed");} 
        else                 { RCLCPP_ERROR(node->get_logger(), "failed to push mission");}
    } else {RCLCPP_ERROR(node->get_logger(), "Failed to receive response");}
}

void clearMission(const std::shared_ptr<DroneController>&node) {
    auto request = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();

    auto future_result = node->clearMission_(request);

    if (hasReplied(node,future_result)) { auto result = future_result.get();
        if (result->success) { RCLCPP_INFO(node->get_logger(), "Mission cleared"); node->reset_wp_();} 
        else                 { RCLCPP_ERROR(node->get_logger(), "failed to clear mission");}
    } else {RCLCPP_ERROR(node->get_logger(), "Failed to receive response");}
}

void waitForWP(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, int seq) {
    while (rclcpp::ok()) {
        if(node->current_wp_() >= seq){
            RCLCPP_INFO(node->get_logger(), "wp reached! :)");
            break;
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void arm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate) {
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;

    if (node->isArmingServiceReady()) {
        auto arm_result = node->arm_(arm_request);
        if (rclcpp::spin_until_future_complete(node, arm_result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Vehicle armed");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to arm vehicle");
        }
    }
    rate.sleep();
}

void disarm(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate) {
    auto disarm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    disarm_request->value = false;

    if (node->isArmingServiceReady()) {
        auto disarm_result = node->arm_(disarm_request);
        if (rclcpp::spin_until_future_complete(node, disarm_result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Vehicle disarmed");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to disarm vehicle");
        }
    }
    rate.sleep();
}

void moveWithLaser(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float x, float y, float z, float max_speed, float tolerance, bool allow_centering_payload, bool allow_centering_ember, float koreksi_max_speed, float timeout) {
    /**
     * Listen, I did so many one liner in this function, I'd advise you to just ask me (palel) if you want to understand it. maybe in the future
     * ill create a better comment snippet for this function, but for now. this will do.
     */
    float P_maju = 1.0;
    float I_maju = 0.0;
    float D_maju = 0.0;
    float total_error_maju = 0;
    float last_error_maju = 0;
    
    float P_koreksi = 1.0;
    float I_koreksi = 0.0;
    float D_koreksi = 0.0;
    float total_error_koreksi = 0;
    float last_error_koreksi = 0;

    geometry_msgs::msg::TwistStamped cmd_vel;
    geometry_msgs::msg::PoseStamped vel_pose;
    sensor_msgs::msg::LaserScan start_scan = node->getCurrentLaserScan();
     
    float x_start = start_scan.ranges[0];
    float y_start = start_scan.ranges[(int)start_scan.ranges.size()/4];
    
    float x_2_edge = (x_start == std::numeric_limits<float>::infinity() || std::isnan(x_start)) ? 1.25 : std::abs(x_start - x);
    float y_2_edge = (y_start == std::numeric_limits<float>::infinity() || std::isnan(y_start)) ? 1.25 : std::abs(y_start - y);
    
    float front = start_scan.ranges[0];
    float left =  start_scan.ranges[(int)start_scan.ranges.size()/4];
    float back =  start_scan.ranges[(int)start_scan.ranges.size()/2];
    float right = start_scan.ranges[(int)start_scan.ranges.size()*0.75];

    float last_front = 4.0;
    float last_left = 4.0;
     
    front = front == std::numeric_limits<float>::infinity() ? 4.0f : front;
    left = left == std::numeric_limits<float>::infinity() ? 2.1f : left;
    back = back == std::numeric_limits<float>::infinity() ? 4.0f : back;
    right = right == std::numeric_limits<float>::infinity() ? 2.1f : right;
     
    float vel_z = (z - node->getCurrentRangefinder().range) / sqrt(x*x + y*y);
     
    //  float max_time = timeout == -1 ? sqrt(x*x + y*y) / (max_speed*0.25) : timeout;
    float max_time = timeout == -1 ? 9999.9999 : timeout;
    rclcpp::Time start_time = node->now();

    bool alt_reached = false;
     
    geometry_msgs::msg::Point target = posee.pose.position + rotatePoint(x, y, getHeading(posee.pose.orientation));

    while(rclcpp::ok())
    {
        float drone_heading = getHeading(posee.pose.orientation);
        sensor_msgs::msg::LaserScan curr_scan = node->getCurrentLaserScan();
        front = curr_scan.ranges[0] != std::numeric_limits<float>::infinity() ? curr_scan.ranges[0] : front;
        left =  (curr_scan.ranges[(int)curr_scan.ranges.size()/4] != std::numeric_limits<float>::infinity() && !std::isnan(curr_scan.ranges[(int)curr_scan.ranges.size()/4])) ? std::min(2.1f, curr_scan.ranges[(int)curr_scan.ranges.size()/4]) : left;
        back =  curr_scan.ranges[(int)curr_scan.ranges.size()/2] != std::numeric_limits<float>::infinity() ? curr_scan.ranges[(int)curr_scan.ranges.size()/2] : back;
        right = (curr_scan.ranges[(int)curr_scan.ranges.size()*0.75] != std::numeric_limits<float>::infinity() && !std::isnan(curr_scan.ranges[(int)curr_scan.ranges.size()*0.75])) ? std::min(2.1f, curr_scan.ranges[(int)curr_scan.ranges.size()*0.75]) : right;
        float err_x = x ? (front != std::numeric_limits<float>::infinity() ? front - x_2_edge : 7.5): 0.0;
        float err_y = y ? (left != std::numeric_limits<float>::infinity() ? left - y_2_edge : 7.5): 0.0;
        
        vel_pose.pose.position.x = x ? limit(max_speed, P_maju*err_x + I_maju*total_error_maju + D_maju*last_error_maju) : (front + back <= 2.85f ? limit(koreksi_max_speed, P_koreksi*(front - back) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : (back == 2.1f && front != 2.1f ? limit(koreksi_max_speed, P_koreksi*(front - 1.25 ) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : (front == 2.1f & back != 2.1f ? limit(koreksi_max_speed, P_koreksi*(1.25 - back) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : 0.0)));
        vel_pose.pose.position.y = y ? limit(max_speed, P_maju*err_y + I_maju*total_error_maju + D_maju*last_error_maju) : (left + right <= 2.85f ? limit(koreksi_max_speed, P_koreksi*(left - right) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : (right == 2.1f && left != 2.1f ? limit(koreksi_max_speed, P_koreksi*(left - 1.25  ) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : (left == 2.1f & right != 2.1f ? limit(koreksi_max_speed, P_koreksi*(1.25 - right) + I_koreksi*total_error_koreksi + D_koreksi*last_error_koreksi) : 0.0)));
        
        RCLCPP_INFO(node->get_logger(), "RANGE | front: %f, left: %f, back : %f, right : %f, err_x: %f, err_y: %f, velx: %f, vely: %f", front, left, back, right, err_x, err_y, vel_pose.pose.position.x, vel_pose.pose.position.y);
        
        float velx_before_rotate = vel_pose.pose.position.x;
        float vely_before_rotate = vel_pose.pose.position.y;
        
        vel_pose.pose.position = rotatePoint(vel_pose.pose.position, drone_heading);
        cmd_vel = cast(vel_pose.pose.position);
        cmd_vel.twist.linear.z = std::abs(z - node->getCurrentRangefinder().range) > 0.15 && alt_reached ? vel_z : 0.8*(z - node->getCurrentRangefinder().range);
        alt_reached = std::abs(z - node->getCurrentRangefinder().range) > 0.15 && !alt_reached;
        
        // RCLCPP_INFO(node->get_logger(), "LOG Z %f | mode: %s, twist: %f, vel_z: %f, rangefinder: %f", z, std::abs(z - node->getCurrentRangefinder().range) > 0.15 ? "vel" : "hold", cmd_vel.twist.linear.z, vel_z, node->getCurrentRangefinder().range);
        // RCLCPP_INFO(node->get_logger(), "bfr rotate: (%f , %f). afr rotate: (%f, %f)", velx_before_rotate, vely_before_rotate, vel_pose.pose.position.x, vel_pose.pose.position.y);
        // RCLCPP_INFO(node->get_logger(), "MOVING WITH LASER | dist2target: %f, time elapsed: %f, front: %f, left: %f, back: %f, right: %f", dist(node->getCurrentLocalPose().pose.position, target), (node->now() - start_time).seconds(), front, left, back, right);

        if(std::isnan(cmd_vel.twist.linear.x) || std::isnan(cmd_vel.twist.linear.y))
        {
            RCLCPP_ERROR(node->get_logger(), "GOT NAN: bfr rotate: (%f , %f). afr rotate: (%f, %f)", velx_before_rotate, vely_before_rotate, vel_pose.pose.position.x, vel_pose.pose.position.y);
            continue;
        }

        if(payloadDetected(node) && allow_centering_payload)
        {
            RCLCPP_INFO(node->get_logger(), "Payload Spotted");
            cmd_vel = zero(cmd_vel);
            // cmd_vel = node->getCurrentVelocity();
            // cmd_vel.twist.linear.x *= -0.75;
            // cmd_vel.twist.linear.y *= -0.75;
            node->publishLocalVelocity(cmd_vel);
            
            rclcpp::spin_some(node); rate.sleep();
            
            break;
        }
        if(emberDetected(node) && allow_centering_ember)
        {
            RCLCPP_INFO(node->get_logger(), "Ember Spotted");
            cmd_vel = zero(cmd_vel);
            // cmd_vel = node->getCurrentVelocity();
            // cmd_vel.twist.linear.x *= -0.75;
            // cmd_vel.twist.linear.y *= -0.75;
            node->publishLocalVelocity(cmd_vel);
            
            rclcpp::spin_some(node); rate.sleep();

            break;
        }
        if(isNear(node->getCurrentLocalPose().pose.position, target, tolerance) || (node->now() - start_time).seconds() >= max_time || (x && front <= 1.25 && (abs(front - last_front) <= 0.25 || last_front == 4.0)) || (y && left <= 1.25 && (abs(left - last_left) <= 0.25 || last_left == 4.0)))
        {
            RCLCPP_INFO(node->get_logger(), "Reached target positionnn, dist2target: %f, time elapsed: %f, front: %f, left: %f", dist(node->getCurrentLocalPose().pose.position, target), (node->now() - start_time).seconds(), front, left);
            // cmd_vel = node->getCurrentVelocity();
            // cmd_vel.twist.linear.x *= -0.25;
            // cmd_vel.twist.linear.y *= -0.25;
            cmd_vel = zero(cmd_vel);
            node->publishLocalVelocity(cmd_vel);

            rclcpp::spin_some(node); rate.sleep();

            break;
        }

        node->publishLocalVelocity(cmd_vel);
        
        total_error_maju += x ? err_x : err_y;
        last_error_maju = x ? err_x : err_y;

        total_error_koreksi += x ? err_y : err_x;
        last_error_koreksi = x ? err_y : err_x;

        if(x && (abs(front - last_front) <= 0.25 || last_front == 4.0))
        {
            last_front = front;
        }
        if(y && (abs(left - last_left) <= 0.25 || last_front == 4.0))
        {
            last_front = front;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    posee = node->getCurrentLocalPose();
}

void holdAtLidarRanges(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, float front, float left, float back, float right, bool allow_centering_payload, bool allow_centering_ember)
{
    RCLCPP_INFO(node->get_logger(), "Moving the drone at lidar ranges of: front: %f, left: %f, back: %f, right: %f", front, left, back, right);
    float err_x, err_y;
    geometry_msgs::msg::Point vel_pose;
    geometry_msgs::msg::TwistStamped cmd_vel;
    while(rclcpp::ok())
    {

        if(front != -1.0)
        {
            err_x = node->getCurrentLaserScan().ranges[0] != std::numeric_limits<float>::infinity() ?  node->getCurrentLaserScan().ranges[0] - front : 0.0;
            vel_pose.x = limit(0.3, 1.5*err_x);
        }
        else if(back != -1.0)
        {
            err_x = node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()/2] != std::numeric_limits<float>::infinity() ?  back - node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()/2] : 0.0;
            vel_pose.x = limit(0.3, 1.5*err_x);
        }
        else
        {
            vel_pose.x = 0.0;
        }

        if(left != -1.0)
        {
            err_y = node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()/4] != std::numeric_limits<float>::infinity() ?  node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()/4] - left : 0.0;
            vel_pose.y = limit(0.3, 1.5*err_y);
        }
        else if(right != -1.0)
        {
            err_y = node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()*3/4] != std::numeric_limits<float>::infinity() ?  right - node->getCurrentLaserScan().ranges[(int)node->getCurrentLaserScan().ranges.size()*3/4] : 0.0;
            vel_pose.y = limit(0.3, 1.5*err_y);
        }
        else
        {
            vel_pose.y = 0.0;
        }

        float drone_heading = getHeading(posee.pose.orientation);
        RCLCPP_INFO(node->get_logger(), "err_x: %f, err_y: %f, cmd_x: %f, cmd_y: %f, heading: %f", err_x, err_y, vel_pose.x, vel_pose.y, drone_heading);
        vel_pose = rotatePoint(vel_pose, drone_heading);
        cmd_vel.twist.linear.x = vel_pose.x;
        cmd_vel.twist.linear.y = vel_pose.y;
        cmd_vel.twist.linear.z = 0.0;
        
        node->publishLocalVelocity(cmd_vel);

        if(payloadDetected(node) && allow_centering_payload)
        {
            RCLCPP_INFO(node->get_logger(), "Payload Spotted");
            break;
        }
        if(emberDetected(node) && allow_centering_ember)
        {
            RCLCPP_INFO(node->get_logger(), "Ember Spotted");
            break;
        }

        if(abs(err_x) < 0.01 && abs(err_y) < 0.01)
        {
            RCLCPP_INFO(node->get_logger(), "Reached target positionnn");
            break;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }
    posee = node->getCurrentLocalPose();
}

float getYawError(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate)
{
    float width = 60.0*DEG2RAD;
    sensor_msgs::msg::LaserScan curr_scan = node->getCurrentLaserScan();
    int left_index = curr_scan.ranges.size()/(2*M_PI) * width/2;
    int right_index = curr_scan.ranges.size() - left_index;

    while(curr_scan.ranges[left_index] < 0.4 || curr_scan.ranges[left_index] > 2.0 || std::isnan(curr_scan.ranges[left_index]) || curr_scan.ranges[left_index] == std::numeric_limits<float>::infinity())
    {
        left_index--;
        if(left_index < 0) {
            RCLCPP_ERROR(node->get_logger(), "depan ga kliatan");
            return 0.0;
        }
    }
    while(curr_scan.ranges[right_index] < 0.4 || curr_scan.ranges[right_index] > 2.0 || std::isnan(curr_scan.ranges[right_index]) || curr_scan.ranges[right_index] == std::numeric_limits<float>::infinity())
    {
        right_index++;
        if(right_index > curr_scan.ranges.size()) {
            RCLCPP_ERROR(node->get_logger(), "depan ga kliatan");
            return 0.0;
        }
    }

    float x1 = curr_scan.ranges[left_index] * cos((curr_scan.ranges.size() / (2*M_PI)) * left_index);
    float y1 = curr_scan.ranges[left_index] * sin((curr_scan.ranges.size() / (2*M_PI)) * left_index);
    float x2 = curr_scan.ranges[right_index] * cos((curr_scan.ranges.size() / (2*M_PI)) * right_index);
    float y2 = curr_scan.ranges[right_index] * sin((curr_scan.ranges.size() / (2*M_PI)) * right_index);

    return atan2(x2 - x1, y1 - y2);

}

void printTitik(std::vector<double>& lat_indoor, std::vector<double>& lon_indoor, std::vector<double>& lat_outdoor, std::vector<double>& lon_outdoor, double lat_takeoff, double lon_takeoff) {
    std::cout << "    lat_takeoff: " << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lat_takeoff << std::endl;
    std::cout << "    lon_takeoff: " << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lon_takeoff << std::endl << std::endl;
    if(lat_indoor.size()) {
        std::cout << "    lat_indoor: [";
        for(int i = 0; i < lat_indoor.size(); i++) {
            std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lat_indoor[i];
            if(i != lat_indoor.size() -1) {std::cout<<", ";}
            else {std::cout << "]"<<std::endl;}
        }
        std::cout << "    lon_indoor: [";
        for(int i = 0; i < lon_indoor.size(); i++) {
            std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lon_indoor[i];
            if(i != lon_indoor.size() -1) {std::cout<<", ";}
            else {std::cout << "]"<<std::endl;}
        }
        std::cout<<std::endl;
    }
    if(lat_outdoor.size())
    {
        std::cout << "    lat_outdoor: [";
        for(int i = 0; i < lat_outdoor.size(); i++) {
            std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lat_outdoor[i];
            if(i != lat_outdoor.size() -1) {std::cout<<", ";}
            else {std::cout << "]"<<std::endl;}
        }
        std::cout << "    lon_outdoor: [";
        for(int i = 0; i < lon_outdoor.size(); i++) {
            std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)<< lon_outdoor[i];
            if(i != lon_outdoor.size() -1) {std::cout<<", ";}
            else {std::cout << "]"<<std::endl;}
        }
        std::cout<<std::endl;
    }
}

void executeLocalWaypointMove(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                               geometry_msgs::msg::PoseStamped &posee, float forward_x, float left_y, 
                               float up_z, float yaw_angle, float tolerance) {
    /**
     * Execute a local waypoint movement by publishing position commands to /mavros/local_position/pose
     * Movement is relative to the current drone's heading (body frame)
     * forward_x: moves in the direction the drone is facing
     * left_y: moves to the left of the drone (perpendicular to heading)
     * For PX4 OFFBOARD: Stream at high rate (20Hz)
     */
    
    // Get current position and heading
    geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    
    // Get current yaw (heading) of the drone
    float current_yaw = getHeading(current_pose.pose.orientation);
    
    RCLCPP_INFO(node->get_logger(), "Current heading: %.2f degrees", current_yaw * 180.0 / M_PI);
    
    // Transform body-frame movement to global frame using current heading
    // In body frame: forward = +x, left = +y
    // Rotate the movement vector by the current yaw to get global coordinates
    float global_x = forward_x * cos(current_yaw) - left_y * sin(current_yaw);
    float global_y = forward_x * sin(current_yaw) + left_y * cos(current_yaw);
    
    // Calculate target position in global frame
    target_pose.pose.position.x = current_pose.pose.position.x + global_x;
    target_pose.pose.position.y = current_pose.pose.position.y + global_y;
    target_pose.pose.position.z = current_pose.pose.position.z + up_z;
    
    // Set orientation (keep current if yaw_angle is 0)
    if (yaw_angle != 0.0) {
        setHeading(target_pose.pose.orientation, yaw_angle);
    } else {
        target_pose.pose.orientation = current_pose.pose.orientation;
    }
    
    // Update timestamp
    target_pose.header.stamp = node->now();
    target_pose.header.frame_id = "map";
    
    RCLCPP_INFO(node->get_logger(), "Moving from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "Body frame: forward=%.2f, left=%.2f -> Global frame: x=%.2f, y=%.2f",
                forward_x, left_y, global_x, global_y);
    
    // PX4 OFFBOARD: Use fast rate for continuous streaming
    rclcpp::Rate fast_rate(20.0); // 20Hz
    
    // Publish target position continuously until reached
    while (rclcpp::ok()) {
        current_pose = node->getCurrentLocalPose();
        
        // Calculate current distance to target
        float current_distance = sqrt(pow(target_pose.pose.position.x - current_pose.pose.position.x, 2) +
                                     pow(target_pose.pose.position.y - current_pose.pose.position.y, 2) +
                                     pow(target_pose.pose.position.z - current_pose.pose.position.z, 2));
        
        // Check if we've reached the target
        if (current_distance < tolerance) {
            RCLCPP_INFO(node->get_logger(), "Waypoint reached! Distance: %.3f m", current_distance);
            break;
        }
        
        // Update timestamp and publish (CRITICAL for PX4)
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        
        // Log progress
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                           "Distance to target: %.3f m", current_distance);
        
        rclcpp::spin_some(node);
        fast_rate.sleep(); // Use fast rate instead of rate parameter
    }
    
    // Update posee to final position
    posee = node->getCurrentLocalPose();
}

void executeLocalWaypointRotation(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate,
                                   geometry_msgs::msg::PoseStamped &posee, float rotation_angle) {
    /**
     * Execute a rotation in place by publishing position commands with updated orientation
     * Rotation is relative to current heading
     * 
     * NOTE: This function uses RADIANS. For easier degree-based rotation, use rotateByDegrees() instead!
     * 
     * PARAMETERS:
     * - rotation_angle: angle in RADIANS
     *   - Positive = RIGHT/Clockwise (e.g., +1.57 rad = 90° right)
     *   - Negative = LEFT/Counter-clockwise (e.g., -1.57 rad = 90° left)
     * 
     * EXAMPLES:
     * - executeLocalWaypointRotation(node, rate, posee, M_PI/2);    // 90° right
     * - executeLocalWaypointRotation(node, rate, posee, -M_PI/2);   // 90° left
     * - executeLocalWaypointRotation(node, rate, posee, M_PI);      // 180° (U-turn)
     */
    
    // Get current pose
    geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    
    // Get current yaw angle
    float current_yaw = getHeading(current_pose.pose.orientation);
    
    // Calculate target yaw angle
    float target_yaw = current_yaw + rotation_angle;
    
    // Normalize to [-pi, pi]
    while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
    while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;
    
    RCLCPP_INFO(node->get_logger(), "Rotating from %.2f degrees to %.2f degrees",
                current_yaw * 180.0 / M_PI, target_yaw * 180.0 / M_PI);
    
    // Keep position the same, only change orientation
    target_pose.pose.position = current_pose.pose.position;
    setHeading(target_pose.pose.orientation, target_yaw);
    target_pose.header.stamp = node->now();
    target_pose.header.frame_id = "map";
    
    // Rotation tolerance (5 degrees in radians)
    float rotation_tolerance = 5.0 * M_PI / 180.0;
    
    // Publish target orientation continuously until reached
    while (rclcpp::ok()) {
        current_pose = node->getCurrentLocalPose();
        current_yaw = getHeading(current_pose.pose.orientation);
        
        // Calculate angle difference
        float angle_diff = target_yaw - current_yaw;
        
        // Normalize angle difference to [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        
        // Check if rotation is complete
        if (fabs(angle_diff) < rotation_tolerance) {
            RCLCPP_INFO(node->get_logger(), "Rotation complete! Current heading: %.2f degrees",
                       current_yaw * 180.0 / M_PI);
            break;
        }
        
        // Update position to current (to maintain position while rotating)
        target_pose.pose.position = current_pose.pose.position;
        target_pose.header.stamp = node->now();
        node->publishLocalPosition(target_pose);
        
        // Log progress
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                           "Angle to target: %.2f degrees", angle_diff * 180.0 / M_PI);
        
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    // Update posee to final pose 
    posee = node->getCurrentLocalPose();
}

// =============================================================

// void executeLocalWaypointRotation(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate,
//                                    geometry_msgs::msg::PoseStamped &posee, float rotation_angle) {
//     /**
//      * Execute a rotation in place with ACTIVE ALTITUDE COMPENSATION
//      * Rotation is relative to current heading
//      * 
//      * IMPROVED: Maintains XY position and actively compensates altitude drift during rotation
//      * 
//      * NOTE: This function uses RADIANS. For easier degree-based rotation, use rotateByDegrees() instead!
//      * 
//      * PARAMETERS:
//      * - rotation_angle: angle in RADIANS
//      *   - Positive = RIGHT/Clockwise (e.g., +1.57 rad = 90° right)
//      *   - Negative = LEFT/Counter-clockwise (e.g., -1.57 rad = 90° left)
//      * 
//      * EXAMPLES:
//      * - executeLocalWaypointRotation(node, rate, posee, M_PI/2);    // 90° right
//      * - executeLocalWaypointRotation(node, rate, posee, -M_PI/2);   // 90° left
//      * - executeLocalWaypointRotation(node, rate, posee, M_PI);      // 180° (U-turn)
//      */
    
//     // Get current pose
//     geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//     geometry_msgs::msg::PoseStamped target_pose = current_pose;
    
//     // Get current yaw angle
//     float current_yaw = getHeading(current_pose.pose.orientation);
    
//     // Calculate target yaw angle
//     float target_yaw = current_yaw + rotation_angle;
    
//     // Normalize to [-pi, pi]
//     while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
//     while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;
    
//     RCLCPP_INFO(node->get_logger(), "Rotating from %.2f degrees to %.2f degrees (WITH ALTITUDE LOCK)",
//                 current_yaw * 180.0 / M_PI, target_yaw * 180.0 / M_PI);
    
//     // ALTITUDE LOCK: Save reference position and altitude from local pose
//     float reference_x = current_pose.pose.position.x;
//     float reference_y = current_pose.pose.position.y;
//     float reference_altitude = current_pose.pose.position.z;
//     float filtered_z = reference_altitude;
//     const float z_filter_alpha = 0.4f;  // Low-pass filter coefficient
    
//     RCLCPP_INFO(node->get_logger(), "Locking position: (%.2f, %.2f, %.3f)", 
//                 reference_x, reference_y, reference_altitude);
    
//     // Set initial target (keep position, change orientation)
//     target_pose.pose.position.x = reference_x;
//     target_pose.pose.position.y = reference_y;
//     target_pose.pose.position.z = reference_altitude;
//     setHeading(target_pose.pose.orientation, target_yaw);
//     target_pose.header.stamp = node->now();
//     target_pose.header.frame_id = "map";
    
//     // Rotation tolerance (5 degrees in radians)
//     float rotation_tolerance = 5.0 * M_PI / 180.0;
    
//     // Publish target orientation continuously until reached
//     rclcpp::Rate fast_rate(20.0); // 20Hz for smooth control
    
//     while (rclcpp::ok()) {
//         current_pose = node->getCurrentLocalPose();
//         current_yaw = getHeading(current_pose.pose.orientation);
        
//         // Calculate angle difference
//         float angle_diff = target_yaw - current_yaw;
        
//         // Normalize angle difference to [-pi, pi]
//         while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
//         while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        
//         // Check if rotation is complete
//         if (fabs(angle_diff) < rotation_tolerance) {
//             RCLCPP_INFO(node->get_logger(), "Rotation complete! Current heading: %.2f degrees",
//                        current_yaw * 180.0 / M_PI);
//             break;
//         }
        
//         // ALTITUDE COMPENSATION: Apply low-pass filter to current Z
//         float current_z = current_pose.pose.position.z;
//         filtered_z = z_filter_alpha * current_z + (1.0f - z_filter_alpha) * filtered_z;
        
//         // Calculate altitude error
//         float altitude_error = reference_altitude - filtered_z;
        
//         // Compensate altitude drift in position setpoint
//         float compensated_z = reference_altitude;
//         if (std::abs(altitude_error) > 0.03f) {  // 3cm threshold
//             // Add compensation to counteract drift
//             compensated_z = reference_altitude + 0.5f * altitude_error;  // Proportional compensation
//         }
        
//         // Update target position (lock XY, compensate Z, update orientation)
//         target_pose.pose.position.x = reference_x;
//         target_pose.pose.position.y = reference_y;
//         target_pose.pose.position.z = compensated_z;
//         setHeading(target_pose.pose.orientation, target_yaw);
//         target_pose.header.stamp = node->now();
        
//         node->publishLocalPosition(target_pose);
        
//         // Log progress
//         RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                            "Angle to target: %.2f°, Alt error: %.3fm", 
//                            angle_diff * 180.0 / M_PI, altitude_error);
        
//         rclcpp::spin_some(node);
//         fast_rate.sleep();
//     }
    
//     // Update posee to final pose 
//     posee = node->getCurrentLocalPose();
// }

void rotateByDegrees(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                     geometry_msgs::msg::PoseStamped &posee, float degrees) {
    /**
     * Wrapper function for executeLocalWaypointRotation() that accepts degrees instead of radians
     * Makes rotation commands more intuitive and easier to use
     * 
     * USAGE:
     * - rotateByDegrees(node, rate, posee, 90.0);   // 90° right (clockwise)
     * - rotateByDegrees(node, rate, posee, -90.0);  // 90° left (counter-clockwise)
     * - rotateByDegrees(node, rate, posee, 180.0);  // Turn around
     */
    
    // Convert degrees to radians
    float radians = degrees * M_PI / 180.0;
    
    RCLCPP_INFO(node->get_logger(), "Rotating %.1f degrees %s...", 
               fabs(degrees), degrees >= 0 ? "RIGHT (clockwise)" : "LEFT (counter-clockwise)");
    
    // Call the original function with radians
    executeLocalWaypointRotation(node, rate, posee, radians);
    //executeLocalWaypointMoveVelocity(node, rate, posee, 0.0, distance, 0.0, 0.0, 0.3, tolerance);
}

void strafeRight(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                 geometry_msgs::msg::PoseStamped &posee, float distance, float tolerance) {
    /**
     * Strafe (move sideways) to the right relative to drone's current heading
     * Positive distance moves to the right
     */
    RCLCPP_INFO(node->get_logger(), "Strafing RIGHT %.2f meters", distance);
    
    // Call executeLocalWaypointMove with negative left_y (right is negative left)
    // forward_x = 0 (no forward/backward movement)
    // left_y = -distance (negative = right)
    // up_z = 0 (no altitude change)
    // yaw_angle = 0 (keep current heading)
    // executeLocalWaypointMove(node, rate, posee, 0.0, -distance, 0.0, 0.0, tolerance); 
    executeLocalWaypointMoveVelocity(node, rate, posee, 0.0, -distance, 0.0, 0.0, 1.2, tolerance);
}

void strafeLeft(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                geometry_msgs::msg::PoseStamped &posee, float distance, float tolerance) {
    /**
     * Strafe (move sideways) to the left relative to drone's current heading
     * Positive distance moves to the left
     */
    RCLCPP_INFO(node->get_logger(), "Strafing LEFT %.2f meters", distance);
    
    // Call executeLocalWaypointMove with positive left_y
    // forward_x = 0 (no forward/backward movement)
    // left_y = +distance (positive = left)
    // up_z = 0 (no altitude change)
    // yaw_angle = 0 (keep current heading)
    // executeLocalWaypointMove(node, rate, posee, 0.0, distance, 0.0, 0.0, tolerance);
    executeLocalWaypointMoveVelocity(node, rate, posee, 0.0, distance, 0.0, 0.0, 1.2, tolerance);
}

bool detectGatePosts(const std::shared_ptr<DroneController>&node, float &left_post_angle, 
                     float &right_post_angle, float &left_post_distance, float &right_post_distance,
                     float gate_width, float max_detection_range) {
    /**
     * Detect two gate posts using 2D lidar scan
     * Algorithm:
     * 1. Get lidar scan data
     * 2. Find clusters of points (obstacles)
     * 3. Identify two clusters that are approximately gate_width apart
     * 4. Calculate angle and distance to each post
     */
    
    sensor_msgs::msg::LaserScan scan = node->getCurrentLaserScan();
    
    if (scan.ranges.empty()) {
        RCLCPP_WARN(node->get_logger(), "Lidar scan is empty!");
        return false;
    }
    
    // Find all obstacles (points closer than max_detection_range)
    struct ObstaclePoint {
        float angle;
        float distance;
        int index;
    };
    
    std::vector<ObstaclePoint> obstacles;
    
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        float range = scan.ranges[i];
        
        // Filter out invalid readings and too far points
        if (std::isfinite(range) && range > scan.range_min && 
            range < scan.range_max && range < max_detection_range) {
            
            float angle = scan.angle_min + i * scan.angle_increment;
            obstacles.push_back({angle, range, (int)i});
        }
    }
    
    if (obstacles.size() < 2) {
        RCLCPP_WARN(node->get_logger(), "Not enough obstacles detected: %zu", obstacles.size());
        return false;
    }
    
    // Cluster obstacles to find distinct posts
    // Simple clustering: find gaps in consecutive points
    std::vector<std::vector<ObstaclePoint>> clusters;
    std::vector<ObstaclePoint> current_cluster;
    
    float cluster_gap_threshold = 0.3; // 0.3 radians (~17 degrees) gap = new cluster (more sensitive)
    
    current_cluster.push_back(obstacles[0]);
    
    for (size_t i = 1; i < obstacles.size(); i++) {
        float angle_diff = obstacles[i].angle - obstacles[i-1].angle;
        
        if (angle_diff > cluster_gap_threshold) {
            // Start new cluster
            if (current_cluster.size() >= 2) { // Minimum 2 points to be a valid post (more sensitive)
                clusters.push_back(current_cluster);
            }
            current_cluster.clear();
        }
        current_cluster.push_back(obstacles[i]);
    }
    
    // Don't forget the last cluster
    if (current_cluster.size() >= 2) {
        clusters.push_back(current_cluster);
    }
    
    if (clusters.size() < 2) {
        RCLCPP_WARN(node->get_logger(), "Only %zu cluster(s) detected, need at least 2 for gate posts", 
                   clusters.size());
        return false;
    }
    
    // Find the centroid of each cluster (average position)
    struct Post {
        float angle;
        float distance;
        float x; // Cartesian x
        float y; // Cartesian y
    };
    
    std::vector<Post> posts;
    
    for (const auto& cluster : clusters) {
        float avg_x = 0.0;
        float avg_y = 0.0;
        
        for (const auto& point : cluster) {
            avg_x += point.distance * cos(point.angle);
            avg_y += point.distance * sin(point.angle);
        }
        
        avg_x /= cluster.size();
        avg_y /= cluster.size();
        
        float post_distance = sqrt(avg_x * avg_x + avg_y * avg_y);
        float post_angle = atan2(avg_y, avg_x);
        
        posts.push_back({post_angle, post_distance, avg_x, avg_y});
    }
    
    // Find two posts that are approximately gate_width apart
    float best_match_error = std::numeric_limits<float>::max();
    int best_left_idx = -1;
    int best_right_idx = -1;
    
    for (size_t i = 0; i < posts.size(); i++) {
        for (size_t j = i + 1; j < posts.size(); j++) {
            // Calculate distance between posts
            float dx = posts[j].x - posts[i].x;
            float dy = posts[j].y - posts[i].y;
            float distance_between = sqrt(dx * dx + dy * dy);
            
            // Check if distance matches gate_width (within 30% tolerance)
            float error = fabs(distance_between - gate_width);
            float tolerance_ratio = 0.3; // 30% tolerance
            
            if (error < gate_width * tolerance_ratio && error < best_match_error) {
                best_match_error = error;
                
                // Determine left and right based on angle
                if (posts[i].angle > posts[j].angle) {
                    best_left_idx = i;
                    best_right_idx = j;
                } else {
                    best_left_idx = j;
                    best_right_idx = i;
                }
            }
        }
    }
    
    if (best_left_idx == -1 || best_right_idx == -1) {
        RCLCPP_WARN(node->get_logger(), 
                   "Could not find two posts matching gate width %.2f m (checked %zu posts)", 
                   gate_width, posts.size());
        return false;
    }
    
    // Output the results
    left_post_angle = posts[best_left_idx].angle;
    left_post_distance = posts[best_left_idx].distance;
    right_post_angle = posts[best_right_idx].angle;
    right_post_distance = posts[best_right_idx].distance;
    
    float detected_width = sqrt(
        pow(posts[best_left_idx].x - posts[best_right_idx].x, 2) +
        pow(posts[best_left_idx].y - posts[best_right_idx].y, 2)
    );
    
    RCLCPP_INFO(node->get_logger(), 
               "Gate detected! Left: angle=%.2f° dist=%.2fm, Right: angle=%.2f° dist=%.2fm, Width=%.2fm",
               left_post_angle * 180.0 / M_PI, left_post_distance,
               right_post_angle * 180.0 / M_PI, right_post_distance,
               detected_width);
    
    return true;
}

void centeringGateLidar(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                        geometry_msgs::msg::PoseStamped &posee, float gate_width, 
                        float tolerance, bool &status, float max_time) {
    /**
     * Center the drone between two gate posts using 2D lidar
     * Strategy:
     * 1. Detect gate posts
     * 2. Calculate center point between posts
     * 3. Calculate offset from drone to center
     * 4. Strafe to center position
     * 5. Repeat until centered within tolerance
     */
    
    RCLCPP_INFO(node->get_logger(), 
               "Centering Lidar ygy... (gate width: %.2f m, tolerance: %.2f m)...",
               gate_width, tolerance);
    
    auto start_time = node->now();
    status = false;
    
    int consecutive_centered_count = 0;
    int required_consecutive = 3; // Require 3 consecutive "centered" readings for stability
    
    while (rclcpp::ok()) {
        // Check timeout
        if ((node->now() - start_time).seconds() > max_time) {
            RCLCPP_WARN(node->get_logger(), "Gate centering timeout after %.1f seconds", max_time);
            status = false;
            return;
        }
        
        // Detect gate posts
        float left_angle, right_angle, left_dist, right_dist;
        if (!detectGatePosts(node, left_angle, right_angle, left_dist, right_dist, gate_width)) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               "Gate tidak kedetek, retrying...");
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // Calculate positions in Cartesian coordinates (body frame)
        float left_x = left_dist * cos(left_angle);
        float left_y = left_dist * sin(left_angle);
        float right_x = right_dist * cos(right_angle);
        float right_y = right_dist * sin(right_angle);
        
        // Calculate center point of the gate
        float center_x = (left_x + right_x) / 2.0;
        float center_y = (left_y + right_y) / 2.0;
        
        // The offset we need to move is just the Y component (lateral offset)
        // Positive center_y means gate center is to the LEFT, so we need to strafe LEFT
        // Negative center_y means gate center is to the RIGHT, so we need to strafe RIGHT
        float lateral_offset = center_y;
        
        RCLCPP_INFO(node->get_logger(), 
                   "offset center gate: %.3f m (forward: %.2f m, lateral: %.2f m)",
                   lateral_offset, center_x, center_y);
        
        // Check if we're centered
        if (fabs(lateral_offset) < tolerance) {
            consecutive_centered_count++;
            RCLCPP_INFO(node->get_logger(), 
                       "Centered! (%.3f m offset, %d/%d consecutive)", 
                       lateral_offset, consecutive_centered_count, required_consecutive);
            
            if (consecutive_centered_count >= required_consecutive) {
                RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
                status = true;
                posee = node->getCurrentLocalPose();
                return;
            }
        } else {
            consecutive_centered_count = 0;
            
            // Calculate strafe distance (limit to prevent overshooting)
            float max_strafe_per_iteration = 0.3; // Maximum 30cm per iteration
            float strafe_distance = lateral_offset;
            
            if (fabs(strafe_distance) > max_strafe_per_iteration) {
                strafe_distance = (strafe_distance > 0) ? max_strafe_per_iteration : -max_strafe_per_iteration;
            }
            
            // Strafe to center
            // Positive offset = gate is to the left, strafe left (positive y)
            // Negative offset = gate is to the right, strafe right (negative y)
            RCLCPP_INFO(node->get_logger(), "Strafing %.3f m to center...", strafe_distance);
            
            executeLocalWaypointMove(node, rate, posee, 0.0, strafe_distance, 0.0, 0.0, 0.1);
        }
        
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    status = false;
}

void centeringGateLidarVelocity(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                                geometry_msgs::msg::PoseStamped &posee, float gate_width, 
                                float tolerance, bool &status, float max_velocity, 
                                float proportional_gain, float max_time) {
    /**
     * Center the drone between two gate posts using 2D lidar with VELOCITY CONTROL
     * This version uses proportional velocity control for smooth, gradual centering
     * 
     * Strategy:
     * 1. Detect gate posts using lidar
     * 2. Calculate lateral offset from gate center
     * 3. Calculate proportional velocity based on offset
     * 4. Publish velocity command continuously
     * 5. When centered, stop and hold position
     * 
     * Advantages over position control:
     * - Smooth, continuous movement (no jerky steps)
     * - Automatic deceleration as approaching center
     * - Better safety with explicit velocity limiting
     * - Real-time feedback and correction
     */
    
    RCLCPP_INFO(node->get_logger(), "Velocity Control Lidar Centering Start ygy... (gate width: %.2f m, tolerance: %.2f m)...", gate_width, tolerance);
    RCLCPP_INFO(node->get_logger(), "Max velocity: %.2f m/s, Proportional gain: %.2f", max_velocity, proportional_gain);
    
    auto start_time = node->now();
    status = false;
    
    int consecutive_centered_count = 0;
    int required_consecutive = 5; // Require 5 consecutive "centered" readings for stability (0.5 sec at 10Hz)
    
    // Get initial position for reference
    geometry_msgs::msg::PoseStamped initial_pose = node->getCurrentLocalPose();
    float initial_z = initial_pose.pose.position.z;
    
    while (rclcpp::ok()) {
        // Check timeout
        if ((node->now() - start_time).seconds() > max_time) {
            RCLCPP_WARN(node->get_logger(), "Gate centering timeout after %.1f seconds", max_time);
            
            // Stop the drone (global frame)
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node->now();
            stop_cmd.header.frame_id = "map"; // Global frame
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.linear.y = 0.0;
            stop_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(stop_cmd);
            
            status = false;
            return;
        }
        
        // Detect gate posts
        float left_angle, right_angle, left_dist, right_dist;
        if (!detectGatePosts(node, left_angle, right_angle, left_dist, right_dist, gate_width)) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               "Gate Ga kedetek, stopping movement...");
            
            // Stop the drone if gate is lost (global frame)
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node->now();
            stop_cmd.header.frame_id = "map"; // Global frame
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.linear.y = 0.0;
            stop_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(stop_cmd);
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // Calculate positions in Cartesian coordinates (body frame)
        float left_x = left_dist * cos(left_angle);
        float left_y = left_dist * sin(left_angle);
        float right_x = right_dist * cos(right_angle);
        float right_y = right_dist * sin(right_angle);
        
        // Calculate center point of the gate
        float center_x = (left_x + right_x) / 2.0;
        float center_y = (left_y + right_y) / 2.0;
        
        // The lateral offset (positive = gate center is to the LEFT)
        float lateral_offset = center_y;
        
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                   "Gate center offset: %.3f m (forward: %.2f m)", lateral_offset, center_x);
        
        // Check if we're centered
        if (fabs(lateral_offset) < tolerance) {
            consecutive_centered_count++;
            
            // Publish zero velocity to stop (in global frame)
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node->now();
            stop_cmd.header.frame_id = "map"; // Global frame
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.linear.y = 0.0;
            stop_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(stop_cmd);
            
            RCLCPP_INFO(node->get_logger(), 
                       "Centered! (%.3f m offset, %d/%d consecutive)", 
                       lateral_offset, consecutive_centered_count, required_consecutive);
            
            if (consecutive_centered_count >= required_consecutive) {
                RCLCPP_INFO(node->get_logger(), "Lidar Vel Control : Gate centering successful");
                
                // Hold position briefly to ensure stability
                holdPosition(node, rate, posee, 1.0);
                
                status = true;
                posee = node->getCurrentLocalPose();
                return;
            }
        } else {
            consecutive_centered_count = 0;
            
            // Calculate proportional velocity in BODY FRAME
            // In body frame: X=forward, Y=left, Z=up
            // Lidar detection already in body frame, so lateral_offset is already correct orientation
            // Positive offset = gate is LEFT, need positive Y velocity (strafe left)
            // Negative offset = gate is RIGHT, need negative Y velocity (strafe right)
            
            float target_velocity_body_y = proportional_gain * lateral_offset;  // Lateral velocity in body frame
            
            // Limit velocity to max_velocity
            if (fabs(target_velocity_body_y) > max_velocity) {
                target_velocity_body_y = (target_velocity_body_y > 0) ? max_velocity : -max_velocity;
            }
            
            // Apply velocity deadband to avoid tiny oscillations
            float velocity_deadband = 0.02; // 2 cm/s deadband
            if (fabs(target_velocity_body_y) < velocity_deadband) {
                target_velocity_body_y = 0.0;
            }
            
            // Get current pose and heading for global frame velocity
            geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
            float current_yaw = getHeading(current_pose.pose.orientation);
            
            // Transform body frame velocity to global frame
            // Body frame: X=forward(0), Y=left(target_velocity_body_y), Z=up
            // Global frame transformation:
            // vx_global = vx_body * cos(yaw) - vy_body * sin(yaw)
            // vy_global = vx_body * sin(yaw) + vy_body * cos(yaw)
            float vx_body = 0.0;  // No forward/backward in body frame
            float vy_body = target_velocity_body_y;  // Lateral strafe in body frame
            
            float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
            float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
            
            // Altitude control (global frame, Z is same in both frames)
            float altitude_error = initial_z - current_pose.pose.position.z;
            float target_velocity_z = 0.0;
            
            // Gentle altitude correction
            if (fabs(altitude_error) > 0.1) {
                target_velocity_z = 0.3 * altitude_error; // Gentle proportional control
                target_velocity_z = std::max(-0.2f, std::min(0.2f, target_velocity_z)); // Limit to ±0.2 m/s
            }
            
            // Publish velocity command in GLOBAL frame (map/odom frame)
            geometry_msgs::msg::TwistStamped vel_cmd;
            vel_cmd.header.stamp = node->now();
            vel_cmd.header.frame_id = "map"; // Global frame for correct orientation
            vel_cmd.twist.linear.x = vx_global;  // Global X velocity
            vel_cmd.twist.linear.y = vy_global;  // Global Y velocity (transformed from body lateral)
            vel_cmd.twist.linear.z = target_velocity_z;  // Altitude hold
            vel_cmd.twist.angular.x = 0.0;
            vel_cmd.twist.angular.y = 0.0;
            vel_cmd.twist.angular.z = 0.0; // No rotation
            
            node->publishLocalVelocity(vel_cmd);
            
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                       "Body Y vel: %.3f m/s | Global vel: [%.3f, %.3f] m/s | Offset: %.3f m | Yaw: %.1f°", 
                       vy_body, vx_global, vy_global, lateral_offset, current_yaw * 180.0 / M_PI);
        }
        
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.stamp = node->now();
    stop_cmd.header.frame_id = "map"; 
    stop_cmd.twist.linear.x = 0.0;
    stop_cmd.twist.linear.y = 0.0;
    stop_cmd.twist.linear.z = 0.0;
    node->publishLocalVelocity(stop_cmd);
    
    status = false;
}

bool detectGatePostsNear(const std::shared_ptr<DroneController>&node, float &left_post_angle, 
                         float &right_post_angle, float &left_post_distance, float &right_post_distance,
                         float gate_width, float max_detection_range, 
                         float forward_zone_min, float max_lateral_angle) {
    /**
     * Detect two gate posts using 2D lidar scan - PRIORITIZE NEAREST FORWARD GATE
     * 
     * NEW STRATEGY for handling multiple gates (side-by-side scenario):
     * 1. Filter posts by forward zone (x > forward_zone_min)
     * 2. Find all gate candidates matching gate_width
     * 3. SCORE each gate based on forward alignment (angle to center)
     * 4. Select gate with BEST alignment (closest to 0° heading)
     * 
     * This ensures drone always targets the gate directly in front,
     * not the one on the side when multiple gates are present.
     */
    
    sensor_msgs::msg::LaserScan scan = node->getCurrentLaserScan();
    
    if (scan.ranges.empty()) {
        RCLCPP_WARN(node->get_logger(), "Lidar scan Kosongg!");
        return false;
    }
    
    // Find all obstacles (points closer than max_detection_range)
    struct ObstaclePoint {
        float angle;
        float distance;
        int index;
    };
    
    std::vector<ObstaclePoint> obstacles;
    
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        float range = scan.ranges[i];
        
        // Filter out invalid readings and too far points
        if (std::isfinite(range) && range > scan.range_min && 
            range < scan.range_max && range < max_detection_range) {
            
            float angle = scan.angle_min + i * scan.angle_increment;
            
            // STRATEGY 1: Filter by forward zone
            float x = range * cos(angle);
            if (x > forward_zone_min) {  // Only consider obstacles in forward zone
                obstacles.push_back({angle, range, (int)i});
            }
        }
    }
    
    if (obstacles.size() < 2) {
        RCLCPP_WARN(node->get_logger(), "Not enough obstacles in forward zone: %zu", obstacles.size());
        return false;
    }
    
    // Cluster obstacles to find distinct posts
    std::vector<std::vector<ObstaclePoint>> clusters;
    std::vector<ObstaclePoint> current_cluster;
    
    // PHASE 1 FIX: Reduced threshold for better detection when close to gate
    float cluster_gap_threshold = 0.2; // 0.2 radians (~11.5 degrees) gap = new cluster (was 0.3)
    
    current_cluster.push_back(obstacles[0]);
    
    for (size_t i = 1; i < obstacles.size(); i++) {
        float angle_diff = obstacles[i].angle - obstacles[i-1].angle;
        
        if (angle_diff > cluster_gap_threshold) {
            // Start new cluster
            if (current_cluster.size() >= 2) {
                clusters.push_back(current_cluster);
            }
            current_cluster.clear();
        }
        current_cluster.push_back(obstacles[i]);
    }
    
    // Don't forget the last cluster
    if (current_cluster.size() >= 2) {
        clusters.push_back(current_cluster);
    }
    
    if (clusters.size() < 2) {
        RCLCPP_WARN(node->get_logger(), "Only %zu cluster(s) detected in forward zone, need at least 2 for gate posts", 
                   clusters.size());
        return false;
    }
    
    // Calculate centroid of each cluster
    struct Post {
        float angle;
        float distance;
        float x; // Cartesian x (forward)
        float y; // Cartesian y (lateral)
    };
    
    std::vector<Post> posts;
    
    for (const auto& cluster : clusters) {
        float avg_x = 0.0;
        float avg_y = 0.0;
        
        for (const auto& point : cluster) {
            avg_x += point.distance * cos(point.angle);
            avg_y += point.distance * sin(point.angle);
        }
        
        avg_x /= cluster.size();
        avg_y /= cluster.size();
        
        float post_distance = sqrt(avg_x * avg_x + avg_y * avg_y);
        float post_angle = atan2(avg_y, avg_x);
        
        posts.push_back({post_angle, post_distance, avg_x, avg_y});
    }
    
    // STRATEGY 2: Find gate candidates and score them by forward alignment
    struct GateCandidate {
        int left_idx;
        int right_idx;
        float center_angle;      // Angle to gate center (0° = straight ahead)
        float forward_distance;  // Average forward distance (x coordinate)
        float width_error;       // Error from expected gate_width
        float alignment_score;   // Lower is better (closer to 0° heading)
    };
    
    std::vector<GateCandidate> gate_candidates;
    
    for (size_t i = 0; i < posts.size(); i++) {
        for (size_t j = i + 1; j < posts.size(); j++) {
            // Calculate distance between posts
            float dx = posts[j].x - posts[i].x;
            float dy = posts[j].y - posts[i].y;
            float distance_between = sqrt(dx * dx + dy * dy);
            
            // PHASE 1 FIX: Increased tolerance for better detection when close to gate
            // Check if distance matches gate_width (within 50% tolerance, was 30%)
            float width_error = fabs(distance_between - gate_width);
            float tolerance_ratio = 0.5; // 50% tolerance (was 0.3)
            
            if (width_error < gate_width * tolerance_ratio) {
                // Valid gate candidate found
                
                // Calculate gate center
                float center_x = (posts[i].x + posts[j].x) / 2.0;
                float center_y = (posts[i].y + posts[j].y) / 2.0;
                float center_angle = atan2(center_y, center_x);
                
                // Calculate forward distance (average x)
                float forward_dist = (posts[i].x + posts[j].x) / 2.0;
                
                // STRATEGY 3: Score based on forward alignment
                // Lower score = better (gate more aligned with heading)
                float alignment_score = fabs(center_angle);
                
                // Skip gates that are too far to the side
                if (alignment_score > max_lateral_angle) {
                    continue;  // Gate is too much to the side, skip it
                }
                
                // Determine left and right based on angle
                int left_idx, right_idx;
                if (posts[i].angle > posts[j].angle) {
                    left_idx = i;
                    right_idx = j;
                } else {
                    left_idx = j;
                    right_idx = i;
                }
                
                gate_candidates.push_back({
                    left_idx, 
                    right_idx, 
                    center_angle, 
                    forward_dist,
                    width_error, 
                    alignment_score
                });
            }
        }
    }
    
    if (gate_candidates.empty()) {
        RCLCPP_WARN(node->get_logger(), 
                   "Gaada point cloud yang bisa di cluster %.2f m depan kosong, gate gakedetek (checked %zu posts)", 
                   gate_width, posts.size());
        return false;
    }
    
    // STRATEGY 4: Select gate with BEST forward alignment (lowest alignment_score)
    auto best_gate = std::min_element(gate_candidates.begin(), gate_candidates.end(),
        [](const GateCandidate& a, const GateCandidate& b) {
            return a.alignment_score < b.alignment_score;
        });
    
    // Output the results
    left_post_angle = posts[best_gate->left_idx].angle;
    left_post_distance = posts[best_gate->left_idx].distance;
    right_post_angle = posts[best_gate->right_idx].angle;
    right_post_distance = posts[best_gate->right_idx].distance;
    
    float detected_width = sqrt(
        pow(posts[best_gate->left_idx].x - posts[best_gate->right_idx].x, 2) +
        pow(posts[best_gate->left_idx].y - posts[best_gate->right_idx].y, 2)
    );
    
    RCLCPP_INFO(node->get_logger(), 
               "NEAREST FORWARD GATE detected! Left: angle=%.2f° dist=%.2fm, Right: angle=%.2f° dist=%.2fm, Width=%.2fm, Center angle=%.2f° (selected from %zu candidates)",
               left_post_angle * 180.0 / M_PI, left_post_distance,
               right_post_angle * 180.0 / M_PI, right_post_distance,
               detected_width, best_gate->center_angle * 180.0 / M_PI, gate_candidates.size());
    
    return true;
}

void centeringGateNear(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, 
                       geometry_msgs::msg::PoseStamped &posee, float gate_width, 
                       float tolerance, bool &status, float max_velocity, 
                       float proportional_gain, float max_time,
                       float forward_zone_min, float max_lateral_angle) {
    /**
     * Center the drone to the NEAREST FORWARD GATE using 2D lidar with VELOCITY CONTROL
     * 
     * NEW STRATEGY for multiple gates scenario:
     * - Prioritizes gate directly in front (best forward alignment)
     * - Filters out gates that are too far to the side
     * - Uses forward zone filtering to ignore side/back obstacles
     * 
     * This version uses proportional velocity control for smooth, gradual centering
     * 
     * Strategy:
     * 1. Detect NEAREST FORWARD gate posts using enhanced detection algorithm
     * 2. Calculate lateral offset from gate center
     * 3. Calculate proportional velocity based on offset
     * 4. Publish velocity command continuously
     * 5. When centered, stop and hold position
     * 
     * Parameters:
     * - forward_zone_min: Minimum forward distance to consider (default 0.5m)
     * - max_lateral_angle: Max angle to gate center (default 45° = 0.785 rad)
     */
    
    RCLCPP_INFO(node->get_logger(), "Starting NEAREST FORWARD gate centering (gate width: %.2f m, tolerance: %.2f m)...", gate_width, tolerance);
    RCLCPP_INFO(node->get_logger(), "Forward zone min: %.2f m, Max lateral angle: %.2f°", 
               forward_zone_min, max_lateral_angle * 180.0 / M_PI);
    RCLCPP_INFO(node->get_logger(), "Max velocity: %.2f m/s, Proportional gain: %.2f", max_velocity, proportional_gain);
    
    auto start_time = node->now();
    status = false;
    
    int consecutive_centered_count = 0;
    int required_consecutive = 5; // Require 5 consecutive "centered" readings for stability (0.5 sec at 10Hz)
    
    // Get initial position for reference
    geometry_msgs::msg::PoseStamped initial_pose = node->getCurrentLocalPose();
    float initial_z = initial_pose.pose.position.z;
    
    while (rclcpp::ok()) {
        // Check timeout
        if ((node->now() - start_time).seconds() > max_time) {
            RCLCPP_WARN(node->get_logger(), "Gate centering timeout after %.1f seconds", max_time);
            
            // Stop the drone (global frame)
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node->now();
            stop_cmd.header.frame_id = "map"; // Global frame
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.linear.y = 0.0;
            stop_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(stop_cmd);
            
            status = false;
            return;
        }
        
        // Detect NEAREST FORWARD gate posts using enhanced algorithm
        float left_angle, right_angle, left_dist, right_dist;
        if (!detectGatePostsNear(node, left_angle, right_angle, left_dist, right_dist, 
                                 gate_width, 10.0, forward_zone_min, max_lateral_angle)) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               "Nearest forward gate not detected, stopping movement...");
            
            // Stop the drone if gate is lost (global frame)
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node->now();
            stop_cmd.header.frame_id = "map"; // Global frame
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.linear.y = 0.0;
            stop_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(stop_cmd);
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // Calculate positions in Cartesian coordinates (body frame)
        float left_x = left_dist * cos(left_angle);
        float left_y = left_dist * sin(left_angle);
        float right_x = right_dist * cos(right_angle);
        float right_y = right_dist * sin(right_angle);
        
        // Calculate center point of the gate
        float center_x = (left_x + right_x) / 2.0;
        float center_y = (left_y + right_y) / 2.0;
        
        // The lateral offset (positive = gate center is to the LEFT)
        float lateral_offset = center_y;
        
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                   "Nearest gate center offset: %.3f m (forward: %.2f m)", lateral_offset, center_x);
        
        // Check if we're centered
        if (fabs(lateral_offset) < tolerance) {
            consecutive_centered_count++;
            
            // Publish zero velocity to stop (in global frame)
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node->now();
            stop_cmd.header.frame_id = "map";
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.linear.y = 0.0;
            stop_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(stop_cmd);
            
            if (consecutive_centered_count >= required_consecutive) {
                RCLCPP_INFO(node->get_logger(), 
                          "Centered on nearest forward gate! (offset: %.3f m, consecutive: %d)", 
                          lateral_offset, consecutive_centered_count);
                status = true;
                return;
            }
            
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                       "Centered, waiting for stability... (%d/%d)", 
                       consecutive_centered_count, required_consecutive);
        } else {
            // Not centered, reset counter
            consecutive_centered_count = 0;
            
            // Calculate target velocity in BODY frame (lateral strafe)
            // BUG FIX: Removed negative sign - it was causing drone to move away from center!
            // In body frame: X=forward, Y=left, Z=up
            // Positive offset = gate is LEFT, need positive Y velocity (strafe left)
            // Negative offset = gate is RIGHT, need negative Y velocity (strafe right)
            // Proportional control: velocity proportional to offset
            float target_velocity_body_y = proportional_gain * lateral_offset;  // ✅ FIXED: No negative sign!
            
            // Limit velocity
            target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
            
            // Get current orientation for frame transformation
            geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
            
            // Extract yaw using getHeading helper function
            float current_yaw = getHeading(current_pose.pose.orientation);
            
            // Transform velocity from body frame to global frame
            // Global frame transformation:
            // vx_global = vx_body * cos(yaw) - vy_body * sin(yaw)
            // vy_global = vx_body * sin(yaw) + vy_body * cos(yaw)
            float vx_body = 0.0;  // No forward/backward in body frame
            float vy_body = target_velocity_body_y;  // Lateral strafe in body frame
            
            float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
            float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
            
            // Altitude control (global frame, Z is same in both frames)
            float altitude_error = initial_z - current_pose.pose.position.z;
            float target_velocity_z = 0.0;
            
            // Gentle altitude correction
            if (fabs(altitude_error) > 0.1) {
                target_velocity_z = 0.3 * altitude_error; // Gentle proportional control
                target_velocity_z = std::max(-0.2f, std::min(0.2f, target_velocity_z)); // Limit to ±0.2 m/s
            }
            
            // Publish velocity command in GLOBAL frame (map/odom frame)
            geometry_msgs::msg::TwistStamped vel_cmd;
            vel_cmd.header.stamp = node->now();
            vel_cmd.header.frame_id = "map"; // Global frame for correct orientation
            vel_cmd.twist.linear.x = vx_global;  // Global X velocity
            vel_cmd.twist.linear.y = vy_global;  // Global Y velocity (transformed from body lateral)
            vel_cmd.twist.linear.z = target_velocity_z;  // Altitude hold
            vel_cmd.twist.angular.x = 0.0;
            vel_cmd.twist.angular.y = 0.0;
            vel_cmd.twist.angular.z = 0.0; // No rotation
            
            node->publishLocalVelocity(vel_cmd);
            
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                       "Body Y vel: %.3f m/s | Global vel: [%.3f, %.3f] m/s | Offset: %.3f m | Yaw: %.1f°", 
                       vy_body, vx_global, vy_global, lateral_offset, current_yaw * 180.0 / M_PI);
        }
        
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    // Final stop command
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.stamp = node->now();
    stop_cmd.header.frame_id = "map"; 
    stop_cmd.twist.linear.x = 0.0;
    stop_cmd.twist.linear.y = 0.0;
    stop_cmd.twist.linear.z = 0.0;
    node->publishLocalVelocity(stop_cmd);
    
    status = false;
}


// ============================================================
// 3D LIDAR GATE CENTERING (Velodyne VLP-16)
// Simple clustering - detects left & right poles only
// ============================================================

struct GatePole3D {
    float x, y, z;           // Centroid position
    float height;            // Height of pole
    int vertical_points;     // Number of vertical points
    int horizontal_points;   // Number of horizontal points
};

void centeringGateLidar3D(const std::shared_ptr<DroneController>&node, 
                          rclcpp::Rate &rate,
                          geometry_msgs::msg::PoseStamped &posee,
                          float gate_width,
                          float lateral_tolerance,
                          float vertical_tolerance,
                          bool &status,
                          float max_velocity,
                          float proportional_gain,
                          float max_time,
                          int min_points_vertical,
                          int min_points_horizontal) {
    /**
     * 3D LIDAR Gate Centering using simple clustering
     * 
     * Algorithm:
     * 1. Get LaserScan from /scan topic (converted from CustomMsg)
     * 2. Filter points in ROI (region of interest)
     * 3. Cluster points spatially (simple grid-based)
     * 4. Validate clusters as gate poles:
     *    - Must be tall (vertical points >= min_points_vertical)
     *    - Must have horizontal spread (>= min_points_horizontal)
     *    - Must be thin (width < 0.3m)
     * 5. Find pair of poles matching gate width
     * 6. Calculate lateral & vertical offset from gate center
     * 7. Apply proportional velocity control
     */
    
    RCLCPP_INFO(node->get_logger(), 
               "=== 3D LIDAR Gate Centering (Velodyne VLP-16) ===");
    RCLCPP_INFO(node->get_logger(),
               "Gate width: %.2fm, Lateral tol: %.2fm, Vertical tol: %.2fm",
               gate_width, lateral_tolerance, vertical_tolerance);
    RCLCPP_INFO(node->get_logger(),
               "Min points - Vertical: %d, Horizontal: %d",
               min_points_vertical, min_points_horizontal);
    
    auto start_time = node->now();
    status = false;
    
    int consecutive_centered_count = 0;
    int required_consecutive = 10;  // 1 second at 10Hz
    
    // Get initial altitude to maintain
    geometry_msgs::msg::PoseStamped initial_pose = node->getCurrentLocalPose();
    float target_z = initial_pose.pose.position.z;
    
    RCLCPP_INFO(node->get_logger(), "Waiting for 3D point cloud data...");
    
    while (rclcpp::ok()) {
        // Check timeout
        if ((node->now() - start_time).seconds() > max_time) {
            RCLCPP_WARN(node->get_logger(), 
                       "Centering timeout after %.1fs", max_time);
            
            // Stop drone
            geometry_msgs::msg::TwistStamped stop_cmd;
            stop_cmd.header.stamp = node->now();
            stop_cmd.header.frame_id = "map";
            stop_cmd.twist.linear.x = 0.0;
            stop_cmd.twist.linear.y = 0.0;
            stop_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(stop_cmd);
            
            status = false;
            return;
        }
        
        // Get current laser scan (3D point cloud from Velodyne)
        sensor_msgs::msg::LaserScan current_scan = node->getCurrentLaserScan();
        
        if (current_scan.ranges.empty()) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                               "No 3D point cloud data, waiting...");
            
            // Hold position
            geometry_msgs::msg::TwistStamped hold_cmd;
            hold_cmd.header.stamp = node->now();
            hold_cmd.header.frame_id = "map";
            hold_cmd.twist.linear.x = 0.0;
            hold_cmd.twist.linear.y = 0.0;
            hold_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(hold_cmd);
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // DEBUG: Log scan info
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                           "Scan: %zu ranges, angle [%.2f to %.2f]", 
                           current_scan.ranges.size(), 
                           current_scan.angle_min, 
                           current_scan.angle_max);
        
        // Process 3D point cloud to find gate poles (left & right only)
        // Simple clustering based on spatial proximity
        std::vector<GatePole3D> detected_poles;
        
        // Convert LaserScan to 3D points and cluster
        // ROI: x[0.5-6m], y[±2.5m], z[0.3-2.5m] (ignore ground and ceiling)
        float roi_x_min = 0.5, roi_x_max = 6.0;
        float roi_y_min = -2.5, roi_y_max = 2.5;
        float roi_z_min = 0.3, roi_z_max = 2.5;  // Ignore ground below 0.3m
        
        // Group points into spatial bins (simple clustering)
        std::map<std::pair<int, int>, std::vector<std::pair<float, float>>> bins;  // (grid_x, grid_y) -> [(x, y)]
        
        float grid_size = 0.2;  // 20cm grid cells
        float angle = current_scan.angle_min;
        
        int total_points = 0;
        int roi_points = 0;
        
        for (size_t i = 0; i < current_scan.ranges.size(); ++i) {
            float range = current_scan.ranges[i];
            
            // Skip invalid ranges
            if (range < current_scan.range_min || range > current_scan.range_max) {
                angle += current_scan.angle_increment;
                continue;
            }
            
            total_points++;
            
            // Convert to Cartesian coordinates
            float x = range * cos(angle);
            float y = range * sin(angle);
            
            // For VLP-16, approximate vertical structure
            // Assume uniform vertical distribution for simplicity
            float z_approx = 1.25;  // Approximate gate center height
            
            // Check if in ROI
            if (x >= roi_x_min && x <= roi_x_max &&
                y >= roi_y_min && y <= roi_y_max &&
                z_approx >= roi_z_min && z_approx <= roi_z_max) {
                
                roi_points++;
                
                // Assign to grid cell
                int grid_x = static_cast<int>(std::floor(x / grid_size));
                int grid_y = static_cast<int>(std::floor(y / grid_size));
                
                bins[{grid_x, grid_y}].push_back({x, y});
            }
            
            angle += current_scan.angle_increment;
        }
        
        // DEBUG: Log point cloud statistics
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                           "Points: total=%d, ROI=%d, bins=%zu", 
                           total_points, roi_points, bins.size());
        
        // Validate clusters as gate poles
        int total_clusters = 0;
        int thin_clusters = 0;
        int vertical_pass = 0;
        
        for (const auto& bin : bins) {
            const auto& points = bin.second;
            
            total_clusters++;
            
            // Check minimum points
            if (points.size() < static_cast<size_t>(min_points_horizontal)) {
                RCLCPP_DEBUG(node->get_logger(), 
                           "Cluster rejected: only %zu points (need %d horizontal)", 
                           points.size(), min_points_horizontal);
                continue;
            }
            
            // Calculate bounding box
            float min_x = 1e6, max_x = -1e6;
            float min_y = 1e6, max_y = -1e6;
            float sum_x = 0, sum_y = 0;
            
            for (const auto& pt : points) {
                min_x = std::min(min_x, pt.first);
                max_x = std::max(max_x, pt.first);
                min_y = std::min(min_y, pt.second);
                max_y = std::max(max_y, pt.second);
                sum_x += pt.first;
                sum_y += pt.second;
            }
            
            float width_x = max_x - min_x;
            float width_y = max_y - min_y;
            
            // Check if pole is thin (< 0.3m in both directions)
            bool is_thin = (width_x < 0.3 && width_y < 0.3);
            
            if (!is_thin) {
                RCLCPP_DEBUG(node->get_logger(), 
                           "Cluster rejected: too wide (%.3f x %.3f m, need <0.3)", 
                           width_x, width_y);
                continue;
            }
            thin_clusters++;
            
            // Check vertical extent (simulate with point density)
            // For real VLP-16, you'd check actual Z distribution
            int vertical_points = points.size() / 4;  // Approximate
            bool has_vertical_structure = (vertical_points >= min_points_vertical);
            
            if (!has_vertical_structure) {
                RCLCPP_DEBUG(node->get_logger(), 
                           "Cluster rejected: not enough vertical points (%d, need %d)", 
                           vertical_points, min_points_vertical);
                continue;
            }
            vertical_pass++;
            
            if (is_thin && has_vertical_structure) {
                GatePole3D pole;
                pole.x = sum_x / points.size();
                pole.y = sum_y / points.size();
                pole.z = 1.25;  // Approximate gate center height
                pole.height = 2.0;  // Approximate
                pole.vertical_points = vertical_points;
                pole.horizontal_points = points.size();
                detected_poles.push_back(pole);
                
                RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               "Pole detected at (%.2f, %.2f) | H:%d V:%d pts | size:%.3fx%.3f", 
                               pole.x, pole.y, pole.horizontal_points, pole.vertical_points,
                               width_x, width_y);
            }
        }
        
        // DEBUG: Log detection statistics
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                           "Clusters: total=%d, thin=%d, vertical_ok=%d, poles=%zu", 
                           total_clusters, thin_clusters, vertical_pass, detected_poles.size());
        
        if (detected_poles.size() < 2) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                               "Found only %zu pole(s), need at least 2",
                               detected_poles.size());
            
            // Hold position
            geometry_msgs::msg::TwistStamped hold_cmd;
            hold_cmd.header.stamp = node->now();
            hold_cmd.header.frame_id = "map";
            hold_cmd.twist.linear.x = 0.0;
            hold_cmd.twist.linear.y = 0.0;
            hold_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(hold_cmd);
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // Find best gate pair (closest pair with correct width)
        GatePole3D* pole_left = nullptr;
        GatePole3D* pole_right = nullptr;
        float min_distance = 1e6;
        float gate_width_tolerance = 0.3;  // ±30cm tolerance
        
        for (size_t i = 0; i < detected_poles.size(); ++i) {
            for (size_t j = i + 1; j < detected_poles.size(); ++j) {
                float dy = detected_poles[j].y - detected_poles[i].y;
                float dx = detected_poles[j].x - detected_poles[i].x;
                float width = std::sqrt(dy*dy + dx*dx);
                
                // Check if width matches expected gate width
                if (std::abs(width - gate_width) <= gate_width_tolerance) {
                    // Check if this is closest gate
                    float avg_x = (detected_poles[i].x + detected_poles[j].x) / 2.0;
                    if (avg_x < min_distance) {
                        min_distance = avg_x;
                        pole_left = &detected_poles[i];
                        pole_right = &detected_poles[j];
                    }
                }
            }
        }
        
        if (!pole_left || !pole_right) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                               "No valid gate found (width mismatch)");
            
            // Hold position
            geometry_msgs::msg::TwistStamped hold_cmd;
            hold_cmd.header.stamp = node->now();
            hold_cmd.header.frame_id = "map";
            hold_cmd.twist.linear.x = 0.0;
            hold_cmd.twist.linear.y = 0.0;
            hold_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(hold_cmd);
            
            consecutive_centered_count = 0;
            rclcpp::spin_some(node);
            rate.sleep();
            continue;
        }
        
        // Ensure pole_left is actually on the left (negative Y)
        if (pole_left->y > pole_right->y) {
            std::swap(pole_left, pole_right);
        }
        
        // Calculate gate center
        float gate_center_x = (pole_left->x + pole_right->x) / 2.0;
        float gate_center_y = (pole_left->y + pole_right->y) / 2.0;
        float gate_center_z = (pole_left->z + pole_right->z) / 2.0;
        
        // Calculate offsets
        // Lateral offset: negative means gate is to the right
        float lateral_offset = gate_center_y;
        // Vertical offset: assume gate center should be at 1.25m
        float vertical_offset = gate_center_z - 1.25;
        float distance = gate_center_x;
        
        // Check if centered
        bool is_lateral_centered = std::abs(lateral_offset) < lateral_tolerance;
        bool is_vertical_centered = std::abs(vertical_offset) < vertical_tolerance;
        bool is_centered = is_lateral_centered && is_vertical_centered;
        
        if (is_centered) {
            consecutive_centered_count++;
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                       "Centered! (%d/%d) | Lateral: %.3fm, Vertical: %.3fm, Dist: %.2fm",
                       consecutive_centered_count, required_consecutive,
                       lateral_offset, vertical_offset, distance);
            
            if (consecutive_centered_count >= required_consecutive) {
                RCLCPP_INFO(node->get_logger(), 
                           "Gate centering successful!");
                
                // Final stop
                geometry_msgs::msg::TwistStamped stop_cmd;
                stop_cmd.header.stamp = node->now();
                stop_cmd.header.frame_id = "map";
                stop_cmd.twist.linear.x = 0.0;
                stop_cmd.twist.linear.y = 0.0;
                stop_cmd.twist.linear.z = 0.0;
                node->publishLocalVelocity(stop_cmd);
                
                // Update posee to final position
                posee = node->getCurrentLocalPose();
                status = true;
                return;
            }
            
            // Still send gentle hold velocity
            geometry_msgs::msg::TwistStamped vel_cmd;
            vel_cmd.header.stamp = node->now();
            vel_cmd.header.frame_id = "map";
            vel_cmd.twist.linear.x = 0.0;
            vel_cmd.twist.linear.y = 0.0;
            vel_cmd.twist.linear.z = 0.0;
            node->publishLocalVelocity(vel_cmd);
            
        } else {
            // Not centered, reset counter and apply corrective velocity
            consecutive_centered_count = 0;
            
            // Calculate proportional velocities (body frame)
            // Lateral: negative offset means gate is to the right, so move right (+Y body)
            float vel_y = -proportional_gain * lateral_offset;
            
            // Vertical: negative offset means gate is above, so move up (+Z)
            float vel_z = -proportional_gain * vertical_offset;
            
            // Limit velocities
            vel_y = std::max(-max_velocity, std::min(max_velocity, vel_y));
            vel_z = std::max(-max_velocity, std::min(max_velocity, vel_z));
            
            // Get current pose and yaw for body->world transformation
            geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
            float current_yaw = getHeading(current_pose.pose.orientation);
            
            // Transform body frame velocity to world frame
            float vel_x_world = -vel_y * sin(current_yaw);  // Body Y -> World X component
            float vel_y_world = vel_y * cos(current_yaw);   // Body Y -> World Y component
            
            // Altitude hold (relative to initial altitude)
            float current_z = current_pose.pose.position.z;
            float z_error = target_z - current_z;
            float vel_z_altitude_hold = 0.3 * z_error;  // Gentle altitude correction
            
            // Combine vertical centering with altitude hold
            float vel_z_combined = vel_z + vel_z_altitude_hold;
            vel_z_combined = std::max(-max_velocity, std::min(max_velocity, vel_z_combined));
            
            // Publish velocity command (world frame)
            geometry_msgs::msg::TwistStamped vel_cmd;
            vel_cmd.header.stamp = node->now();
            vel_cmd.header.frame_id = "map";
            vel_cmd.twist.linear.x = vel_x_world;
            vel_cmd.twist.linear.y = vel_y_world;
            vel_cmd.twist.linear.z = vel_z_combined;
            vel_cmd.twist.angular.x = 0.0;
            vel_cmd.twist.angular.y = 0.0;
            vel_cmd.twist.angular.z = 0.0;
            
            node->publishLocalVelocity(vel_cmd);
            
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                       "Centering: Lat=%.3fm (L:%d|R:%d pts), Vert=%.3fm, Dist=%.2fm | Vel:[%.3f,%.3f,%.3f]",
                       lateral_offset, pole_left->horizontal_points, pole_right->horizontal_points,
                       vertical_offset, distance,
                       vel_x_world, vel_y_world, vel_z_combined);
        }
        
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    // Final stop if loop exits unexpectedly
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.stamp = node->now();
    stop_cmd.header.frame_id = "map";
    stop_cmd.twist.linear.x = 0.0;
    stop_cmd.twist.linear.y = 0.0;
    stop_cmd.twist.linear.z = 0.0;
    node->publishLocalVelocity(stop_cmd);
    
    status = false;
}

// ============================================================================
// SIMPLE GATE CENTERING - Grid Binning Approach (Similar to 2D Lidar)
// ============================================================================

// void centeringGateLivoxSimple(
//     const std::shared_ptr<DroneController>&node,
//     rclcpp::Rate &rate,
//     geometry_msgs::msg::PoseStamped &posee,
//     float gate_width,
//     float tolerance,
//     bool &status,
//     float max_velocity,
//     float proportional_gain,
//     float max_time,
//     bool test_mode)
// {
//     if (test_mode) {
//         RCLCPP_INFO(node->get_logger(), "=== LIVOX CENTERING TEST MODE ===");
//     } else {
//         RCLCPP_INFO(node->get_logger(), "=== Starting SIMPLE Livox Gate Centering ===");
//     }
//     RCLCPP_INFO(node->get_logger(), "Expected gate width: %.2fm, Tolerance: %.3fm", 
//                 gate_width, tolerance);
//     if (!test_mode) {
//         RCLCPP_INFO(node->get_logger(), "Strategy: Grid binning + peak detection (O(n) complexity)");
//     }

//     float detection_range_min = 1.5f;
//     float detection_range_max = 2.5f;
//     float roi_lateral = 2.0f;
//     int min_cluster_points = 10;
    
//     node->get_parameter("gate_centering.detection_range_min", detection_range_min);
//     node->get_parameter("gate_centering.detection_range_max", detection_range_max);
//     node->get_parameter("gate_centering.roi_lateral", roi_lateral);
//     node->get_parameter("gate_centering.min_cluster_points", min_cluster_points);
    
//     // ROI parameters
//     const float x_min = detection_range_min;  // Forward distance min
//     const float x_max = detection_range_max;  // Forward distance max
//     const float y_min = -roi_lateral; // Lateral min (left)
//     const float y_max = roi_lateral;  // Lateral max (right)
//     const float z_min = 0.0f;  // Height min
//     const float z_max = 2.0f;  // Height max
    
//     // Grid binning parameters
//     const int num_bins = 60;  // 40 bins for 4m lateral range
//     const float bin_size = (y_max - y_min) / num_bins;  // 0.1m per bin
//     const int min_points_per_pole = min_cluster_points;  // Minimum points to consider as pole
    
//     // Gate validation parameters
//     const float gate_width_min = 0.8f;
//     const float gate_width_max = 2.5f;
    
//     RCLCPP_INFO(node->get_logger(), "ROI: X[%.1f-%.1fm] Y[%.1f-%.1fm] Z[%.1f-%.1fm]",
//                 x_min, x_max, y_min, y_max, z_min, z_max);
//     RCLCPP_INFO(node->get_logger(), "Grid: %d bins @ %.2fm, Min points/pole: %d",
//                 num_bins, bin_size, min_points_per_pole);
    
//     // Subscribe to Livox Custom LiDAR data
//     livox_ros_driver2::msg::CustomMsg::SharedPtr latest_cloud = nullptr;
    
//     auto lidar_sub = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10,
//         [&latest_cloud](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//             latest_cloud = msg;
//         });
    
//     RCLCPP_INFO(node->get_logger(), "Subscribing to Livox CustomMsg on /livox/lidar");

//     // Centering control variables
//     auto start_time = node->now();
//     int consecutive_centered_count = 0;
//     const int required_centered_frames = 10; // 1 second at 10Hz
//     status = false;
    
//     // Get initial altitude for stability
//     geometry_msgs::msg::PoseStamped initial_pose = node->getCurrentLocalPose();
//     float initial_z = initial_pose.pose.position.z;
    
//     // PX4 CRITICAL: Prepare hold position for streaming when waiting
//     geometry_msgs::msg::PoseStamped hold_pose = initial_pose;
    
//     while (rclcpp::ok()) {
//         // Check timeout
//         auto elapsed = (node->now() - start_time).seconds();
//         if (elapsed > max_time) {
//             RCLCPP_ERROR(node->get_logger(), "Gate centering timeout after %.1fs", elapsed);
//             break;
//         }
        
//         if (!latest_cloud || latest_cloud->point_num == 0) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
//                 "Waiting for Livox CustomMsg data on /livox/lidar...");
            
//             // PX4 CRITICAL: MUST stream setpoint even when waiting for data!
//             if (!test_mode) {
//                 hold_pose = node->getCurrentLocalPose();
//                 hold_pose.header.stamp = node->now();
//                 node->publishLocalPosition(hold_pose);
//             }
            
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 1: Extract and filter points by ROI using CustomMsg =====
        
//         // CustomMsg structure:
//         // - header: std_msgs/Header
//         // - timebase: uint64
//         // - point_num: uint32 (total number of points)
//         // - lidar_id: uint8
//         // - rsvd: uint8[3]
//         // - points: CustomPoint[] array
//         //
//         // CustomPoint structure:
//         // - offset_time: uint32
//         // - x, y, z: float32
//         // - reflectivity: uint8
//         // - tag: uint8
//         // - line: uint8
        
//         // ===== STEP 2: Grid binning - count points per lateral bin =====
        
//         int bins[num_bins] = {0};  // Initialize all bins to 0
//         int total_roi_points = 0;
        
//         // Access points directly from CustomMsg
//         const auto& points = latest_cloud->points;
//         uint32_t num_points = latest_cloud->point_num;
        
//         // Validate point count
//         if (num_points != static_cast<uint32_t>(points.size())) {
//             RCLCPP_WARN(node->get_logger(), 
//                 "CustomMsg point_num (%u) doesn't match points array size (%zu)", 
//                 latest_cloud->point_num, points.size());
//             num_points = std::min(num_points, static_cast<uint32_t>(points.size()));
//         }
        
//         for (size_t i = 0; i < num_points; ++i) {
//             const auto& point = points[i];
            
//             float x = point.x;
//             float y = point.y;
//             float z = point.z;
            
//             // Skip invalid points
//             if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
//             // Filter by ROI
//             if (x >= x_min && x <= x_max &&
//                 y >= y_min && y <= y_max &&
//                 z >= z_min && z <= z_max) {
                
//                 // Calculate bin index
//                 int bin_idx = static_cast<int>((y - y_min) / bin_size);
                
//                 // Clamp to valid range
//                 if (bin_idx >= 0 && bin_idx < num_bins) {
//                     bins[bin_idx]++;
//                     total_roi_points++;
//                 }
//             }
//         }
        
//         if (total_roi_points < min_points_per_pole * 2) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Insufficient points in ROI: %d (need %d)", 
//                 total_roi_points, min_points_per_pole * 2);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.z = vz_global;  // Maintain altitude
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 3: Find 2 peak bins (left & right poles) =====
        
//         // Find first peak (highest bin count)
//         int first_peak_idx = -1;
//         int first_peak_count = 0;
        
//         for (int i = 0; i < num_bins; i++) {
//             if (bins[i] > first_peak_count) {
//                 first_peak_count = bins[i];
//                 first_peak_idx = i;
//             }
//         }
        
//         if (first_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "First peak too weak: %d points (need %d)", 
//                 first_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.z = vz_global;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // Find second peak (must be at least 10 bins away from first peak)
//         int second_peak_idx = -1;
//         int second_peak_count = 0;
//         const int min_peak_separation = 10;  // ~1.0m minimum separation
        
//         for (int i = 0; i < num_bins; i++) {
//             // Must be far enough from first peak
//             if (std::abs(i - first_peak_idx) < min_peak_separation) continue;
            
//             if (bins[i] > second_peak_count) {
//                 second_peak_count = bins[i];
//                 second_peak_idx = i;
//             }
//         }
        
//         if (second_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Second peak too weak: %d points (need %d)", 
//                 second_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.z = vz_global;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 4: Determine left & right poles =====
        
//         int left_idx = (first_peak_idx < second_peak_idx) ? first_peak_idx : second_peak_idx;
//         int right_idx = (first_peak_idx < second_peak_idx) ? second_peak_idx : first_peak_idx;
        
//         float left_y = y_min + (left_idx + 0.5f) * bin_size;  // Center of bin
//         float right_y = y_min + (right_idx + 0.5f) * bin_size;
        
//         float detected_width = right_y - left_y;
        
//         // ===== STEP 5: Validate gate width =====
        
//         if (detected_width < gate_width_min || detected_width > gate_width_max) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Invalid gate width: %.2fm (expected %.2f-%.2fm)",
//                 detected_width, gate_width_min, gate_width_max);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.z = vz_global;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 6: Calculate lateral offset =====
        
//         float center_y = (left_y + right_y) / 2.0f;
//         float lateral_error = center_y;  // Positive = gate to the right
        
//         // ===== STEP 7: Check if centered =====
        
//         bool is_centered = (std::abs(lateral_error) < tolerance);
        
//         if (is_centered) {
//             consecutive_centered_count++;
            
//             if (test_mode) {
//                 RCLCPP_INFO(node->get_logger(),
//                     "CENTERED! Frame %d/%d | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error);
//             } else {
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "CENTERED! Frame %d/%d | Gate: [L:%.2fm R:%.2fm W:%.2fm] | Err: %.3fm | Pts[L:%d R:%d]",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error,
//                     bins[left_idx], bins[right_idx]);
//             }
            
//             // Stop lateral movement when centered, but maintain altitude!
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                
//                 // Altitude control (gentle correction to maintain initial altitude)
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {  // Tighter tolerance when centered
//                     vz_global = 0.4f * altitude_error;  // Slightly more aggressive
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;  // No forward/backward
//                 stop_cmd.twist.linear.y = 0.0;  // No lateral movement
//                 stop_cmd.twist.linear.z = vz_global;  // Maintain altitude!
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             if (consecutive_centered_count >= required_centered_frames) {
//                 if (test_mode) {
//                     RCLCPP_INFO(node->get_logger(), "TEST MODE: Centering validation SUCCESSFUL!");
//                 } else {
//                     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESSFUL!");
//                 }
//                 status = true;
//                 break;
//             }
//         } else {
//             consecutive_centered_count = 0;
            
//             // ===== STEP 8: Calculate and publish velocity command =====
            
//             // In BODY frame: X=forward, Y=left, Z=up
//             // lateral_error > 0 means gate is to the RIGHT, need to move RIGHT (negative Y in body frame... wait no!)
//             // Actually in Livox/ROS convention:
//             // - Y positive = left side
//             // - Y negative = right side
//             // So if gate center has positive Y, it's on the left, we need to move left (positive Y velocity)
//             // If gate center has negative Y, it's on the right, we need to move right (negative Y velocity)
//             // Simple: velocity should match the error sign (no negative!)
            
//             float target_velocity_body_y = proportional_gain * lateral_error;  // No negative sign!
            
//             // Apply velocity limits
//             target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
            
//             if (test_mode) {
//                 std::string direction = (lateral_error > 0) ? "RIGHT" : "LEFT";
                
//                 RCLCPP_INFO(node->get_logger(),
//                     "NOT CENTERED | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm %s | Vel:%.3fm/s",
//                     left_y, right_y, detected_width, lateral_error, direction.c_str(), target_velocity_body_y);
//             }
            
//             if (!test_mode) {
//                 // Get current pose for frame transformation
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float current_yaw = getHeading(current_pose.pose.orientation);
                
//                 // Transform velocity from BODY frame to GLOBAL frame (map)
//                 // This is critical for correct movement regardless of drone orientation!
//                 // Global frame transformation:
//                 // vx_global = vx_body * cos(yaw) - vy_body * sin(yaw)
//                 // vy_global = vx_body * sin(yaw) + vy_body * cos(yaw)
//                 float vx_body = 0.0;  // No forward/backward during centering
//                 float vy_body = target_velocity_body_y;
                
//                 float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
//                 float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
                
//                 // Altitude control (gentle correction)
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.1f) {
//                     vz_global = 0.3f * altitude_error;  // Gentle proportional control
//                     vz_global = std::max(-0.2f, std::min(0.2f, vz_global));
//                 }
                
//                 // Publish velocity command in GLOBAL frame (map/odom)
//                 geometry_msgs::msg::TwistStamped twist_cmd;
//                 twist_cmd.header.stamp = node->now();
//                 twist_cmd.header.frame_id = "map";  // Global frame for correct orientation
//                 twist_cmd.twist.linear.x = vx_global;
//                 twist_cmd.twist.linear.y = vy_global;
//                 twist_cmd.twist.linear.z = vz_global;
                
//                 node->publishLocalVelocity(twist_cmd);
                
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "Centering: Gate[L:%.2f R:%.2f W:%.2fm] | Err:%.3fm | Vel_body_y:%.3f | Vel_map[x:%.3f y:%.3f] | Yaw:%.1f° | Pts[L:%d R:%d]",
//                     left_y, right_y, detected_width, lateral_error, target_velocity_body_y, 
//                     vx_global, vy_global, current_yaw * 180.0 / M_PI,
//                     bins[left_idx], bins[right_idx]);
//             }
//         }
        
//         rclcpp::spin_some(node);
//         rate.sleep();
//     }
    
//     // Final stop
//     if (!test_mode) {
//         geometry_msgs::msg::TwistStamped stop_cmd;
//         stop_cmd.header.stamp = node->now();
//         stop_cmd.header.frame_id = "map";
//         node->publishLocalVelocity(stop_cmd);
//     }
    
//     if (status) {
//         if (test_mode) {
//             RCLCPP_INFO(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation COMPLETED ===");
//         } else {
//             RCLCPP_INFO(node->get_logger(), "=== SIMPLE Livox Gate Centering COMPLETED ===");
//         }
//     } else {
//         if (test_mode) {
//             RCLCPP_ERROR(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation FAILED ===");
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "=== SIMPLE Livox Gate Centering FAILED ===");
//         }
//     }
// }

// ============================================================================
// = OLD VERSION FOR MAKAI YAW CORRECTION GATE
// ============================================================================

// void centeringGateLivoxSimple(
//     const std::shared_ptr<DroneController>&node,
//     rclcpp::Rate &rate,
//     geometry_msgs::msg::PoseStamped &posee,
//     float gate_width,
//     float tolerance,
//     bool &status,
//     float max_velocity,
//     float proportional_gain,
//     float max_time,
//     bool test_mode)
// {
//     if (test_mode) {
//         RCLCPP_INFO(node->get_logger(), "=== LIVOX CENTERING TEST MODE ===");
//     } else {
//         RCLCPP_INFO(node->get_logger(), "=== Starting SIMPLE Livox Gate Centering ===");
//     }
//     RCLCPP_INFO(node->get_logger(), "Expected gate width: %.2fm, Tolerance: %.3fm", 
//                 gate_width, tolerance);
//     if (!test_mode) {
//         RCLCPP_INFO(node->get_logger(), "Strategy: Grid binning + peak detection (O(n) complexity)");
//     }

//     float detection_range_min = 1.5f;
//     float detection_range_max = 2.5f;
//     float roi_lateral = 2.0f;
//     int min_cluster_points = 10;
    
//     node->get_parameter("gate_centering.detection_range_min", detection_range_min);
//     node->get_parameter("gate_centering.detection_range_max", detection_range_max);
//     node->get_parameter("gate_centering.roi_lateral", roi_lateral);
//     node->get_parameter("gate_centering.min_cluster_points", min_cluster_points);
    
//     // ROI parameters
//     const float x_min = detection_range_min;  // Forward distance min
//     const float x_max = detection_range_max;  // Forward distance max
//     const float y_min = -roi_lateral; // Lateral min (left)
//     const float y_max = roi_lateral;  // Lateral max (right)
//     const float z_min = 0.0f;  // Height min
//     const float z_max = 2.0f;  // Height max
    
//     // Grid binning parameters
//     const int num_bins = 60;  // 40 bins for 4m lateral range
//     const float bin_size = (y_max - y_min) / num_bins;  // 0.1m per bin
//     const int min_points_per_pole = min_cluster_points;  // Minimum points to consider as pole
    
//     // Gate validation parameters
//     const float gate_width_min = 0.8f;
//     const float gate_width_max = 2.5f;
    
//     RCLCPP_INFO(node->get_logger(), "ROI: X[%.1f-%.1fm] Y[%.1f-%.1fm] Z[%.1f-%.1fm]",
//                 x_min, x_max, y_min, y_max, z_min, z_max);
//     RCLCPP_INFO(node->get_logger(), "Grid: %d bins @ %.2fm, Min points/pole: %d",
//                 num_bins, bin_size, min_points_per_pole);
    
//     // Subscribe to Livox Custom LiDAR data
//     livox_ros_driver2::msg::CustomMsg::SharedPtr latest_cloud = nullptr;
    
//     auto lidar_sub = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10,
//         [&latest_cloud](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//             latest_cloud = msg;
//         });
    
//     RCLCPP_INFO(node->get_logger(), "Subscribing to Livox CustomMsg on /livox/lidar");

//     // Centering control variables
//     auto start_time = node->now();
//     int consecutive_centered_count = 0;
//     const int required_centered_frames = 10; // 1 second at 10Hz
//     status = false;
    
//     // Get initial altitude for stability
//     geometry_msgs::msg::PoseStamped initial_pose = node->getCurrentLocalPose();
//     float initial_z = initial_pose.pose.position.z;
    
//     // PX4 CRITICAL: Prepare hold position for streaming when waiting
//     geometry_msgs::msg::PoseStamped hold_pose = initial_pose;
    
//     while (rclcpp::ok()) {
//         // Check timeout
//         auto elapsed = (node->now() - start_time).seconds();
//         if (elapsed > max_time) {
//             RCLCPP_ERROR(node->get_logger(), "Gate centering timeout after %.1fs", elapsed);
//             break;
//         }
        
//         if (!latest_cloud || latest_cloud->point_num == 0) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
//                 "Waiting for Livox CustomMsg data on /livox/lidar...");
            
//             // PX4 CRITICAL: MUST stream setpoint even when waiting for data!
//             if (!test_mode) {
//                 hold_pose = node->getCurrentLocalPose();
//                 hold_pose.header.stamp = node->now();
//                 node->publishLocalPosition(hold_pose);
//             }
            
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 1: Extract and filter points by ROI using CustomMsg =====
        
//         // CustomMsg structure:
//         // - header: std_msgs/Header
//         // - timebase: uint64
//         // - point_num: uint32 (total number of points)
//         // - lidar_id: uint8
//         // - rsvd: uint8[3]
//         // - points: CustomPoint[] array
//         //
//         // CustomPoint structure:
//         // - offset_time: uint32
//         // - x, y, z: float32
//         // - reflectivity: uint8
//         // - tag: uint8
//         // - line: uint8
        
//         // ===== STEP 2: Grid binning - count points per lateral bin =====
        
//         int bins[num_bins] = {0}; 
//         float bins_x_sum[num_bins] = {0.0f}; 
//         int total_roi_points = 0;
        
//         // Access points directly from CustomMsg
//         const auto& points = latest_cloud->points;
//         uint32_t num_points = latest_cloud->point_num;
        
//         // Validate point count
//         if (num_points != static_cast<uint32_t>(points.size())) {
//             RCLCPP_WARN(node->get_logger(), 
//                 "CustomMsg point_num (%u) doesn't match points array size (%zu)", 
//                 latest_cloud->point_num, points.size());
//             num_points = std::min(num_points, static_cast<uint32_t>(points.size()));
//         }
        
//         for (size_t i = 0; i < num_points; ++i) {
//             const auto& point = points[i];
            
//             float x = point.x;
//             float y = point.y;
//             float z = point.z;
            
//             // Skip invalid points
//             if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
//             // Filter by ROI
//             if (x >= x_min && x <= x_max &&
//                 y >= y_min && y <= y_max &&
//                 z >= z_min && z <= z_max) {
                
//                 // Calculate bin index
//                 int bin_idx = static_cast<int>((y - y_min) / bin_size);
                
//                 // Clamp to valid range
//                 if (bin_idx >= 0 && bin_idx < num_bins) {
//                     bins[bin_idx]++;
//                     bins_x_sum[bin_idx] += x;
//                     total_roi_points++;
//                 }
//             }
//         }
        
//         if (total_roi_points < min_points_per_pole * 2) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Insufficient points in ROI: %d (need %d)", 
//                 total_roi_points, min_points_per_pole * 2);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.z = vz_global;  // Maintain altitude
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 3: Find 2 peak bins (left & right poles) =====
        
//         // Find first peak (highest bin count)
//         int first_peak_idx = -1;
//         int first_peak_count = 0;
        
//         for (int i = 0; i < num_bins; i++) {
//             if (bins[i] > first_peak_count) {
//                 first_peak_count = bins[i];
//                 first_peak_idx = i;
//             }
//         }
        
//         if (first_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "First peak too weak: %d points (need %d)", 
//                 first_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.z = vz_global;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // Find second peak (must be at least 10 bins away from first peak)
//         int second_peak_idx = -1;
//         int second_peak_count = 0;
//         const int min_peak_separation = 10;  // ~1.0m minimum separation
        
//         for (int i = 0; i < num_bins; i++) {
//             // Must be far enough from first peak
//             if (std::abs(i - first_peak_idx) < min_peak_separation) continue;
            
//             if (bins[i] > second_peak_count) {
//                 second_peak_count = bins[i];
//                 second_peak_idx = i;
//             }
//         }
        
//         if (second_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Second peak too weak: %d points (need %d)", 
//                 second_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.z = vz_global;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 4: Determine left & right poles =====
        
//         int left_idx = (first_peak_idx < second_peak_idx) ? first_peak_idx : second_peak_idx;
//         int right_idx = (first_peak_idx < second_peak_idx) ? second_peak_idx : first_peak_idx;
        
//         float left_y = y_min + (left_idx + 0.5f) * bin_size;  // Center of bin
//         float right_y = y_min + (right_idx + 0.5f) * bin_size;
        
//         float detected_width = right_y - left_y;

//         float left_x = bins_x_sum[left_idx] / bins[left_idx];
//         float right_x = bins_x_sum[right_idx] / bins[right_idx];
//         float x_diff = right_x - left_x;
//         float gate_angle = atan2(x_diff, detected_width);
        
//         // ===== STEP 5: Validate gate width =====
        
//         if (detected_width < gate_width_min || detected_width > gate_width_max) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Invalid gate width: %.2fm (expected %.2f-%.2fm)",
//                 detected_width, gate_width_min, gate_width_max);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.z = vz_global;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 6: Calculate lateral offset =====
        
//         float center_y = (left_y + right_y) / 2.0f;
//         float lateral_error = center_y;  // Positive = gate to the right
        
//         // ===== STEP 7: Check if centered =====
//         const float yaw_tolerance = 0.087f;  // 5 degrees in radians
//         float yaw_error = normalize_angle(gate_angle);
//         bool is_yaw_aligned = (std::abs(yaw_error) < yaw_tolerance);
        
//         bool is_laterally_centered = (std::abs(lateral_error) < tolerance);
//         bool is_centered = is_laterally_centered && is_yaw_aligned;
        
//         if (is_centered) {
//             consecutive_centered_count++;
            
//             if (test_mode) {
//                 RCLCPP_INFO(node->get_logger(),
//                     "CENTERED! Frame %d/%d | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error);
//             } else {
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "CENTERED! Frame %d/%d | Gate: [L:%.2fm R:%.2fm W:%.2fm] | Err: %.3fm | Pts[L:%d R:%d]",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error,
//                     bins[left_idx], bins[right_idx]);
//             }
            
//             // Stop lateral movement when centered, but maintain altitude!
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                
//                 // Altitude control (gentle correction to maintain initial altitude)
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {  // Tighter tolerance when centered
//                     vz_global = 0.4f * altitude_error;  // Slightly more aggressive
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }

//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;  // No forward/backward
//                 stop_cmd.twist.linear.y = 0.0;  // No lateral movement
//                 stop_cmd.twist.linear.z = vz_global;  // Maintain altitude!
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             if (consecutive_centered_count >= required_centered_frames) {
//                 if (test_mode) {
//                     RCLCPP_INFO(node->get_logger(), "TEST MODE: Centering validation SUCCESSFUL!");
//                 } else {
//                     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESSFUL!");
//                 }
//                 status = true;
//                 break;
//             }
//         } else {
//             consecutive_centered_count = 0;
            
//             // ===== STEP 8: Calculate and publish velocity command =====
            
//             // In BODY frame: X=forward, Y=left, Z=up
//             // lateral_error > 0 means gate is to the RIGHT, need to move RIGHT (negative Y in body frame... wait no!)
//             // Actually in Livox/ROS convention:
//             // - Y positive = left side
//             // - Y negative = right side
//             // So if gate center has positive Y, it's on the left, we need to move left (positive Y velocity)
//             // If gate center has negative Y, it's on the right, we need to move right (negative Y velocity)
//             // Simple: velocity should match the error sign (no negative!)
            
//             float target_velocity_body_y = proportional_gain * lateral_error;  // No negative sign!
            
//             // Apply velocity limits
//             target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
            
//             if (test_mode) {
//                 std::string direction = (lateral_error > 0) ? "RIGHT" : "LEFT";
                
//                 RCLCPP_INFO(node->get_logger(),
//                     "NOT CENTERED | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm %s | Vel:%.3fm/s",
//                     left_y, right_y, detected_width, lateral_error, direction.c_str(), target_velocity_body_y);
//             }
            
//             if (!test_mode) {
//                 // Get current pose for frame transformation
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float current_yaw = getHeading(current_pose.pose.orientation);
                
//                 // Transform velocity from BODY frame to GLOBAL frame (map)
//                 // This is critical for correct movement regardless of drone orientation!
//                 // Global frame transformation:
//                 // vx_global = vx_body * cos(yaw) - vy_body * sin(yaw)
//                 // vy_global = vx_body * sin(yaw) + vy_body * cos(yaw)
//                 float vx_body = 0.0;  // No forward/backward during centering
//                 float vy_body = target_velocity_body_y;
                
//                 float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
//                 float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
                
//                 // Altitude control (gentle correction)
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.1f) {
//                     vz_global = 0.3f * altitude_error;  // Gentle proportional control
//                     vz_global = std::max(-0.2f, std::min(0.2f, vz_global));
//                 }
                
//                 const float kp_yaw = 0.3f;  // Proportional gain for yaw
//                 const float max_angular_vel = 0.3f;  // rad/s
//                 float angular_z = kp_yaw * yaw_error;
//                 angular_z = std::max(-max_angular_vel, std::min(max_angular_vel, angular_z));

//                 // Publish velocity command in GLOBAL frame (map/odom)
//                 geometry_msgs::msg::TwistStamped twist_cmd;
//                 twist_cmd.header.stamp = node->now();
//                 twist_cmd.header.frame_id = "map";  // Global frame for correct orientation
//                 twist_cmd.twist.linear.x = vx_global;
//                 twist_cmd.twist.linear.y = vy_global;
//                 twist_cmd.twist.linear.z = vz_global;
//                 twist_cmd.twist.angular.z = -angular_z;  
                
//                 node->publishLocalVelocity(twist_cmd);
                
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "Centering: Gate[L:%.2f R:%.2f W:%.2fm] | Err:%.3fm | Vel_body_y:%.3f | Vel_map[x:%.3f y:%.3f] | Yaw:%.1f° | Pts[L:%d R:%d]",
//                     left_y, right_y, detected_width, lateral_error, target_velocity_body_y, 
//                     vx_global, vy_global, current_yaw * 180.0 / M_PI,
//                     bins[left_idx], bins[right_idx]);
//             }
//         }
        
//         rclcpp::spin_some(node);
//         rate.sleep();
//     }
    
//     // Final stop
//     if (!test_mode) {
//         geometry_msgs::msg::TwistStamped stop_cmd;
//         stop_cmd.header.stamp = node->now();
//         stop_cmd.header.frame_id = "map";
//         node->publishLocalVelocity(stop_cmd);
//     }
    
//     if (status) {
//         if (test_mode) {
//             RCLCPP_INFO(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation COMPLETED ===");
//         } else {
//             RCLCPP_INFO(node->get_logger(), "=== SIMPLE Livox Gate Centering COMPLETED ===");
//         }
//     } else {
//         if (test_mode) {
//             RCLCPP_ERROR(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation FAILED ===");
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "=== SIMPLE Livox Gate Centering FAILED ===");
//         }
//     }
// }

// void centeringGateLivoxSimple(
//     const std::shared_ptr<DroneController>&node,
//     rclcpp::Rate &rate,
//     geometry_msgs::msg::PoseStamped &posee,
//     float gate_width,
//     float tolerance,
//     bool &status,
//     float max_velocity,
//     float proportional_gain,
//     float max_time,
//     bool test_mode)
// {
//     if (test_mode) {
//         RCLCPP_INFO(node->get_logger(), "=== LIVOX CENTERING TEST MODE ===");
//     } else {
//         RCLCPP_INFO(node->get_logger(), "=== Starting SIMPLE Livox Gate Centering ===");
//     }
//     RCLCPP_INFO(node->get_logger(), "Expected gate width: %.2fm, Tolerance: %.3fm", 
//                 gate_width, tolerance);
//     if (!test_mode) {
//         RCLCPP_INFO(node->get_logger(), "Strategy: Grid binning + peak detection (O(n) complexity)");
//     }

//     float detection_range_min = 1.5f;
//     float detection_range_max = 2.5f;
//     float roi_lateral = 2.0f;
//     int min_cluster_points = 10;
    
//     node->get_parameter("gate_centering.detection_range_min", detection_range_min);
//     node->get_parameter("gate_centering.detection_range_max", detection_range_max);
//     node->get_parameter("gate_centering.roi_lateral", roi_lateral);
//     node->get_parameter("gate_centering.min_cluster_points", min_cluster_points);
    
//     // ROI parameters
//     const float x_min = detection_range_min;  // Forward distance min
//     const float x_max = detection_range_max;  // Forward distance max
//     const float y_min = -roi_lateral; // Lateral min (left)
//     const float y_max = roi_lateral;  // Lateral max (right)
//     const float z_min = 0.0f;  // Height min
//     const float z_max = 2.0f;  // Height max
    
//     // Grid binning parameters
//     const int num_bins = 60;  // 40 bins for 4m lateral range
//     const float bin_size = (y_max - y_min) / num_bins;  // 0.1m per bin
//     const int min_points_per_pole = min_cluster_points;  // Minimum points to consider as pole
    
//     // Gate validation parameters
//     const float gate_width_min = 0.8f;
//     const float gate_width_max = 2.5f;
    
//     RCLCPP_INFO(node->get_logger(), "ROI: X[%.1f-%.1fm] Y[%.1f-%.1fm] Z[%.1f-%.1fm]",
//                 x_min, x_max, y_min, y_max, z_min, z_max);
//     RCLCPP_INFO(node->get_logger(), "Grid: %d bins @ %.2fm, Min points/pole: %d",
//                 num_bins, bin_size, min_points_per_pole);
    
//     // Subscribe to Livox Custom LiDAR data
//     livox_ros_driver2::msg::CustomMsg::SharedPtr latest_cloud = nullptr;
    
//     auto lidar_sub = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10,
//         [&latest_cloud](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//             latest_cloud = msg;
//         });
    
//     RCLCPP_INFO(node->get_logger(), "Subscribing to Livox CustomMsg on /livox/lidar");

//     // Centering control variables
//     auto start_time = node->now();
//     int consecutive_centered_count = 0;
//     const int required_centered_frames = 10; // 1 second at 10Hz
//     status = false;
    
//     // Get initial altitude for stability
//     geometry_msgs::msg::PoseStamped initial_pose = node->getCurrentLocalPose();
//     float initial_z = initial_pose.pose.position.z;
    
//     // PX4 CRITICAL: Prepare hold position for streaming when waiting
//     geometry_msgs::msg::PoseStamped hold_pose = initial_pose;
    
//     while (rclcpp::ok()) {
//         // Check timeout
//         auto elapsed = (node->now() - start_time).seconds();
//         if (elapsed > max_time) {
//             RCLCPP_ERROR(node->get_logger(), "Gate centering timeout after %.1fs", elapsed);
//             break;
//         }
        
//         if (!latest_cloud || latest_cloud->point_num == 0) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
//                 "Waiting for Livox CustomMsg data on /livox/lidar...");
            
//             // PX4 CRITICAL: MUST stream setpoint even when waiting for data!
//             if (!test_mode) {
//                 hold_pose = node->getCurrentLocalPose();
//                 hold_pose.header.stamp = node->now();
//                 node->publishLocalPosition(hold_pose);
//             }
            
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 1: Extract and filter points by ROI using CustomMsg =====
        
//         // CustomMsg structure:
//         // - header: std_msgs/Header
//         // - timebase: uint64
//         // - point_num: uint32 (total number of points)
//         // - lidar_id: uint8
//         // - rsvd: uint8[3]
//         // - points: CustomPoint[] array
//         //
//         // CustomPoint structure:
//         // - offset_time: uint32
//         // - x, y, z: float32
//         // - reflectivity: uint8
//         // - tag: uint8
//         // - line: uint8
        
//         // ===== STEP 2: Grid binning - count points per lateral bin =====
        
//         int bins[num_bins] = {0};  // Initialize all bins to 0
//         float bins_x_sum[num_bins] = {0.0f};  // Sum of X coordinates per bin for angle calculation
//         int total_roi_points = 0;
        
//         // Access points directly from CustomMsg
//         const auto& points = latest_cloud->points;
//         uint32_t num_points = latest_cloud->point_num;
        
//         // Validate point count
//         if (num_points != static_cast<uint32_t>(points.size())) {
//             RCLCPP_WARN(node->get_logger(), 
//                 "CustomMsg point_num (%u) doesn't match points array size (%zu)", 
//                 latest_cloud->point_num, points.size());
//             num_points = std::min(num_points, static_cast<uint32_t>(points.size()));
//         }
        
//         for (size_t i = 0; i < num_points; ++i) {
//             const auto& point = points[i];
            
//             float x = point.x;
//             float y = point.y;
//             float z = point.z;
            
//             // Skip invalid points
//             if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
//             // Filter by ROI
//             if (x >= x_min && x <= x_max &&
//                 y >= y_min && y <= y_max &&
//                 z >= z_min && z <= z_max) {
                
//                 // Calculate bin index
//                 int bin_idx = static_cast<int>((y - y_min) / bin_size);
                
//                 // Clamp to valid range
//                 if (bin_idx >= 0 && bin_idx < num_bins) {
//                     bins[bin_idx]++;
//                     bins_x_sum[bin_idx] += x;  // Accumulate X for angle calculation
//                     total_roi_points++;
//                 }
//             }
//         }
        
//         if (total_roi_points < min_points_per_pole * 2) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Insufficient points in ROI: %d (need %d)", 
//                 total_roi_points, min_points_per_pole * 2);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;  // Maintain altitude
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // No yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 3: Find 2 peak bins (left & right poles) =====
        
//         // Find first peak (highest bin count)
//         int first_peak_idx = -1;
//         int first_peak_count = 0;
        
//         for (int i = 0; i < num_bins; i++) {
//             if (bins[i] > first_peak_count) {
//                 first_peak_count = bins[i];
//                 first_peak_idx = i;
//             }
//         }
        
//         if (first_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "First peak too weak: %d points (need %d)", 
//                 first_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // No yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // Find second peak (must be at least 10 bins away from first peak)
//         int second_peak_idx = -1;
//         int second_peak_count = 0;
//         const int min_peak_separation = 10;  // ~1.0m minimum separation
        
//         for (int i = 0; i < num_bins; i++) {
//             // Must be far enough from first peak
//             if (std::abs(i - first_peak_idx) < min_peak_separation) continue;
            
//             if (bins[i] > second_peak_count) {
//                 second_peak_count = bins[i];
//                 second_peak_idx = i;
//             }
//         }
        
//         if (second_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Second peak too weak: %d points (need %d)", 
//                 second_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // No yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 4: Determine left & right poles =====
        
//         int left_idx = (first_peak_idx < second_peak_idx) ? first_peak_idx : second_peak_idx;
//         int right_idx = (first_peak_idx < second_peak_idx) ? second_peak_idx : first_peak_idx;
        
//         float left_y = y_min + (left_idx + 0.5f) * bin_size;  // Center of bin
//         float right_y = y_min + (right_idx + 0.5f) * bin_size;
        
//         float detected_width = right_y - left_y;

//         // Calculate gate angle from X positions of poles
//         float left_x = bins_x_sum[left_idx] / bins[left_idx];
//         float right_x = bins_x_sum[right_idx] / bins[right_idx];
//         float x_diff = right_x - left_x;
//         float gate_angle = atan2(x_diff, detected_width);
        
//         // ===== STEP 5: Validate gate width =====
        
//         if (detected_width < gate_width_min || detected_width > gate_width_max) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Invalid gate width: %.2fm (expected %.2f-%.2fm)",
//                 detected_width, gate_width_min, gate_width_max);
            
//             // Stop lateral movement but maintain altitude
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // No yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 6: Calculate lateral offset =====
        
//         float center_y = (left_y + right_y) / 2.0f;
//         float lateral_error = center_y;  // Positive = gate to the right
        
//         // ===== STEP 7: Check if centered (lateral AND yaw aligned) =====
//         const float yaw_tolerance = 0.087f;  // 5 degrees in radians
//         float yaw_error = normalize_angle(gate_angle);
//         bool is_yaw_aligned = (std::abs(yaw_error) < yaw_tolerance);
        
//         bool is_laterally_centered = (std::abs(lateral_error) < tolerance);
//         bool is_centered = is_laterally_centered && is_yaw_aligned;
        
//         if (is_centered) {
//             consecutive_centered_count++;
            
//             if (test_mode) {
//                 RCLCPP_INFO(node->get_logger(),
//                     "CENTERED! Frame %d/%d | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error);
//             } else {
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "CENTERED! Frame %d/%d | Gate: [L:%.2fm R:%.2fm W:%.2fm] | Err: %.3fm YawErr:%.1f° | Pts[L:%d R:%d]",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI,
//                     bins[left_idx], bins[right_idx]);
//             }
            
//             // Stop lateral movement when centered, but maintain altitude!
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
                
//                 // Altitude control (gentle correction to maintain initial altitude)
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {  // Tighter tolerance when centered
//                     vz_global = 0.4f * altitude_error;  // Slightly more aggressive
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;  // No forward/backward
//                 stop_cmd.twist.linear.y = 0.0;  // No lateral movement
//                 stop_cmd.twist.linear.z = vz_global;  // Maintain altitude!
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // CRITICAL: Lock yaw when centered
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             if (consecutive_centered_count >= required_centered_frames) {
//                 if (test_mode) {
//                     RCLCPP_INFO(node->get_logger(), "TEST MODE: Centering validation SUCCESSFUL!");
//                 } else {
//                     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESSFUL!");
//                 }
//                 status = true;
//                 break;
//             }
//         } else {
//             consecutive_centered_count = 0;
            
//             // ===== STEP 8: Calculate and publish velocity command =====
            
//             // In BODY frame: X=forward, Y=left, Z=up
//             // lateral_error > 0 means gate is to the RIGHT, need to move RIGHT (negative Y in body frame... wait no!)
//             // Actually in Livox/ROS convention:
//             // - Y positive = left side
//             // - Y negative = right side
//             // So if gate center has positive Y, it's on the left, we need to move left (positive Y velocity)
//             // If gate center has negative Y, it's on the right, we need to move right (negative Y velocity)
//             // Simple: velocity should match the error sign (no negative!)
            
//             float target_velocity_body_y = proportional_gain * lateral_error;  // No negative sign!
            
//             // Apply velocity limits
//             target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
            
//             if (test_mode) {
//                 std::string direction = (lateral_error > 0) ? "RIGHT" : "LEFT";
                
//                 RCLCPP_INFO(node->get_logger(),
//                     "NOT CENTERED | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm %s | Vel:%.3fm/s",
//                     left_y, right_y, detected_width, lateral_error, direction.c_str(), target_velocity_body_y);
//             }
            
//             if (!test_mode) {
//                 // Get current pose for frame transformation
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float current_yaw = getHeading(current_pose.pose.orientation);
                
//                 // Transform velocity from BODY frame to GLOBAL frame (map)
//                 // This is critical for correct movement regardless of drone orientation!
//                 // Global frame transformation:
//                 // vx_global = vx_body * cos(yaw) - vy_body * sin(yaw)
//                 // vy_global = vx_body * sin(yaw) + vy_body * cos(yaw)
//                 float vx_body = 0.0;  // No forward/backward during centering
//                 float vy_body = target_velocity_body_y;
                
//                 float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
//                 float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
                
//                 // Altitude control (gentle correction)
//                 float altitude_error = initial_z - current_pose.pose.position.z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.1f) {
//                     vz_global = 0.3f * altitude_error;  // Gentle proportional control
//                     vz_global = std::max(-0.2f, std::min(0.2f, vz_global));
//                 }
                
//                 // Yaw control (proportional correction to align with gate)
//                 const float kp_yaw = 0.3f;  // Proportional gain for yaw
//                 const float max_angular_vel = 0.3f;  // rad/s
//                 float angular_z = kp_yaw * yaw_error;
//                 angular_z = std::max(-max_angular_vel, std::min(max_angular_vel, angular_z));

//                 // Publish velocity command in GLOBAL frame (map/odom)
//                 geometry_msgs::msg::TwistStamped twist_cmd;
//                 twist_cmd.header.stamp = node->now();
//                 twist_cmd.header.frame_id = "map";  // Global frame for correct orientation
//                 twist_cmd.twist.linear.x = vx_global;
//                 twist_cmd.twist.linear.y = vy_global;
//                 twist_cmd.twist.linear.z = vz_global;
//                 twist_cmd.twist.angular.x = 0.0;
//                 twist_cmd.twist.angular.y = 0.0;
//                 twist_cmd.twist.angular.z = -angular_z;  // Yaw correction
                
//                 node->publishLocalVelocity(twist_cmd);
                
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "Centering: Gate[L:%.2f R:%.2f W:%.2fm] | LateralErr:%.3fm YawErr:%.1f° | Vel[x:%.3f y:%.3f z:%.3f ω:%.3f] | Pts[L:%d R:%d]",
//                     left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI,
//                     vx_global, vy_global, vz_global, angular_z,
//                     bins[left_idx], bins[right_idx]);
//             }
//         }
        
//         rclcpp::spin_some(node);
//         rate.sleep();
//     }
    
//     // Final stop
//     if (!test_mode) {
//         geometry_msgs::msg::TwistStamped stop_cmd;
//         stop_cmd.header.stamp = node->now();
//         stop_cmd.header.frame_id = "map";
//         stop_cmd.twist.linear.x = 0.0;
//         stop_cmd.twist.linear.y = 0.0;
//         stop_cmd.twist.linear.z = 0.0;
//         stop_cmd.twist.angular.x = 0.0;
//         stop_cmd.twist.angular.y = 0.0;
//         stop_cmd.twist.angular.z = 0.0;
//         node->publishLocalVelocity(stop_cmd);
//     }
    
//     if (status) {
//         if (test_mode) {
//             RCLCPP_INFO(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation COMPLETED ===");
//         } else {
//             RCLCPP_INFO(node->get_logger(), "=== SIMPLE Livox Gate Centering COMPLETED ===");
//         }
//     } else {
//         if (test_mode) {
//             RCLCPP_ERROR(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation FAILED ===");
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "=== SIMPLE Livox Gate Centering FAILED ===");
//         }
//     }
// }
// 

// =============================================================
// FIXX INI POLLLL
// =============================================================

// void centeringGateLivoxSimple(
//     const std::shared_ptr<DroneController>&node,
//     rclcpp::Rate &rate,
//     geometry_msgs::msg::PoseStamped &posee,
//     float gate_width,
//     float tolerance,
//     bool &status,
//     float max_velocity,
//     float proportional_gain,
//     float max_time,
//     bool test_mode)
// {
//     if (test_mode) {
//         RCLCPP_INFO(node->get_logger(), "=== LIVOX CENTERING TEST MODE ===");
//     } else {
//         RCLCPP_INFO(node->get_logger(), "=== Starting SIMPLE Livox Gate Centering ===");
//     }
//     RCLCPP_INFO(node->get_logger(), "Expected gate width: %.2fm, Tolerance: %.3fm", 
//                 gate_width, tolerance);
//     if (!test_mode) {
//         RCLCPP_INFO(node->get_logger(), "Strategy: Grid binning + peak detection (O(n) complexity)");
//     }

//     float detection_range_min = 1.5f;
//     float detection_range_max = 2.5f;
//     float roi_lateral = 2.0f;
//     int min_cluster_points = 10;
    
//     node->get_parameter("gate_centering.detection_range_min", detection_range_min);
//     node->get_parameter("gate_centering.detection_range_max", detection_range_max);
//     node->get_parameter("gate_centering.roi_lateral", roi_lateral);
//     node->get_parameter("gate_centering.min_cluster_points", min_cluster_points);
    
//     // ROI parameters
//     const float x_min = detection_range_min;  // Forward distance min
//     const float x_max = detection_range_max;  // Forward distance max
//     const float y_min = -roi_lateral; // Lateral min (left)
//     const float y_max = roi_lateral;  // Lateral max (right)
//     const float z_min = 0.0f;  // Height min
//     const float z_max = 2.0f;  // Height max
    
//     // Grid binning parameters
//     const int num_bins = 60;  // 40 bins for 4m lateral range
//     const float bin_size = (y_max - y_min) / num_bins;  // 0.1m per bin
//     const int min_points_per_pole = min_cluster_points;  // Minimum points to consider as pole
    
//     // Gate validation parameters
//     const float gate_width_min = 0.8f;
//     const float gate_width_max = 2.5f;
    
//     RCLCPP_INFO(node->get_logger(), "ROI: X[%.1f-%.1fm] Y[%.1f-%.1fm] Z[%.1f-%.1fm]",
//                 x_min, x_max, y_min, y_max, z_min, z_max);
//     RCLCPP_INFO(node->get_logger(), "Grid: %d bins @ %.2fm, Min points/pole: %d",
//                 num_bins, bin_size, min_points_per_pole);
    
//     // Subscribe to Livox Custom LiDAR data
//     livox_ros_driver2::msg::CustomMsg::SharedPtr latest_cloud = nullptr;
    
//     auto lidar_sub = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10,
//         [&latest_cloud](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//             latest_cloud = msg;
//         });
    
//     RCLCPP_INFO(node->get_logger(), "Subscribing to Livox CustomMsg on /livox/lidar");

//     // Centering control variables
//     auto start_time = node->now();
//     int consecutive_centered_count = 0;
//     const int required_centered_frames = 1; // 1 second at 10Hz
//     status = false;
    
//     // ALTITUDE STABILITY IMPROVEMENT: Use rangefinder directly (more stable than local pose Z)
//     sensor_msgs::msg::Range initial_rangefinder = node->getCurrentRangefinder();
//     float initial_rangefinder_distance = initial_rangefinder.range;
    
//     // Low-pass filter for rangefinder (reduce noise from surface texture)
//     float filtered_rangefinder = initial_rangefinder_distance;
//     const float rangefinder_filter_alpha = 0.4f;  // Slightly more responsive for rangefinder
    
//     // Reference altitude tracking (to compensate drift over time)
//     float reference_altitude = initial_rangefinder_distance;
//     auto last_reference_update = node->now();
//     const double reference_update_interval = 5.0;  // Update reference every 5 seconds when stable
//     int stable_altitude_count = 0;
//     const int required_stable_frames = 20;  // 2 seconds of stability before updating reference
    
//     RCLCPP_INFO(node->get_logger(), "Initial rangefinder altitude: %.3fm", initial_rangefinder_distance);
    
//     // PX4 CRITICAL: Prepare hold position for streaming when waiting
//     geometry_msgs::msg::PoseStamped hold_pose = node->getCurrentLocalPose();
    
//     while (rclcpp::ok()) {
//         // Check timeout
//         auto elapsed = (node->now() - start_time).seconds();
//         if (elapsed > max_time) {
//             RCLCPP_ERROR(node->get_logger(), "Gate centering timeout after %.1fs", elapsed);
//             break;
//         }
        
//         if (!latest_cloud || latest_cloud->point_num == 0) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
//                 "Waiting for Livox CustomMsg data on /livox/lidar...");
            
//             // PX4 CRITICAL: MUST stream setpoint even when waiting for data!
//             if (!test_mode) {
//                 hold_pose = node->getCurrentLocalPose();
//                 hold_pose.header.stamp = node->now();
//                 node->publishLocalPosition(hold_pose);
//             }
            
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 1: Extract and filter points by ROI using CustomMsg =====
        
//         // CustomMsg structure:
//         // - header: std_msgs/Header
//         // - timebase: uint64
//         // - point_num: uint32 (total number of points)
//         // - lidar_id: uint8
//         // - rsvd: uint8[3]
//         // - points: CustomPoint[] array
//         //
//         // CustomPoint structure:
//         // - offset_time: uint32
//         // - x, y, z: float32
//         // - reflectivity: uint8
//         // - tag: uint8
//         // - line: uint8
        
//         // ===== STEP 2: Grid binning - count points per lateral bin =====
        
//         int bins[num_bins] = {0};  // Initialize all bins to 0
//         float bins_x_sum[num_bins] = {0.0f};  // Sum of X coordinates per bin for angle calculation
//         int total_roi_points = 0;
        
//         // Access points directly from CustomMsg
//         const auto& points = latest_cloud->points;
//         uint32_t num_points = latest_cloud->point_num;
        
//         // Validate point count
//         if (num_points != static_cast<uint32_t>(points.size())) {
//             RCLCPP_WARN(node->get_logger(), 
//                 "CustomMsg point_num (%u) doesn't match points array size (%zu)", 
//                 latest_cloud->point_num, points.size());
//             num_points = std::min(num_points, static_cast<uint32_t>(points.size()));
//         }
        
//         for (size_t i = 0; i < num_points; ++i) {
//             const auto& point = points[i];
            
//             float x = point.x;
//             float y = point.y;
//             float z = point.z;
            
//             // Skip invalid points
//             if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
//             // Filter by ROI
//             if (x >= x_min && x <= x_max &&
//                 y >= y_min && y <= y_max &&
//                 z >= z_min && z <= z_max) {
                
//                 // Calculate bin index
//                 int bin_idx = static_cast<int>((y - y_min) / bin_size);
                
//                 // Clamp to valid range
//                 if (bin_idx >= 0 && bin_idx < num_bins) {
//                     bins[bin_idx]++;
//                     bins_x_sum[bin_idx] += x;  // Accumulate X for angle calculation
//                     total_roi_points++;
//                 }
//             }
//         }
        
//         if (total_roi_points < min_points_per_pole * 2) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Insufficient points in ROI: %d (need %d)", 
//                 total_roi_points, min_points_per_pole * 2);
            
//             // Stop lateral movement but maintain altitude with STABILITY FIX
//             if (!test_mode) {
//                 // RANGEFINDER-BASED ALTITUDE CONTROL (more stable than local pose Z)
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
                
//                 // Apply low-pass filter to reduce noise
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 // Calculate altitude error based on filtered rangefinder
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
                
//                 // Larger deadband for rangefinder (accounts for surface texture noise)
//                 if (std::abs(altitude_error) > 0.10f) {  // 10cm deadband (rangefinder typical noise)
//                     vz_global = 0.3f * altitude_error;  // Moderate gain for stability
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));  // Conservative max velocity
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;  // Maintain altitude
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // No yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 3: Find 2 peak bins (left & right poles) =====
        
//         // Find first peak (highest bin count)
//         int first_peak_idx = -1;
//         int first_peak_count = 0;
        
//         for (int i = 0; i < num_bins; i++) {
//             if (bins[i] > first_peak_count) {
//                 first_peak_count = bins[i];
//                 first_peak_idx = i;
//             }
//         }
        
//         if (first_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "First peak too weak: %d points (need %d)", 
//                 first_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude with STABILITY FIX
//             if (!test_mode) {
//                 // RANGEFINDER-BASED ALTITUDE CONTROL
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.10f) {  // 10cm deadband
//                     vz_global = 0.3f * altitude_error;
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // No yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // Find second peak (must be at least 10 bins away from first peak)
//         int second_peak_idx = -1;
//         int second_peak_count = 0;
//         const int min_peak_separation = 10;  // ~1.0m minimum separation
        
//         for (int i = 0; i < num_bins; i++) {
//             // Must be far enough from first peak
//             if (std::abs(i - first_peak_idx) < min_peak_separation) continue;
            
//             if (bins[i] > second_peak_count) {
//                 second_peak_count = bins[i];
//                 second_peak_idx = i;
//             }
//         }
        
//         if (second_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Second peak too weak: %d points (need %d)", 
//                 second_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude with STABILITY FIX
//             if (!test_mode) {
//                 // RANGEFINDER-BASED ALTITUDE CONTROL
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.10f) {  // 10cm deadband
//                     vz_global = 0.3f * altitude_error;
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // No yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 4: Determine left & right poles =====
        
//         int left_idx = (first_peak_idx < second_peak_idx) ? first_peak_idx : second_peak_idx;
//         int right_idx = (first_peak_idx < second_peak_idx) ? second_peak_idx : first_peak_idx;
        
//         float left_y = y_min + (left_idx + 0.5f) * bin_size;  // Center of bin
//         float right_y = y_min + (right_idx + 0.5f) * bin_size;
        
//         float detected_width = right_y - left_y;

//         // Calculate gate angle from X positions of poles
//         float left_x = bins_x_sum[left_idx] / bins[left_idx];
//         float right_x = bins_x_sum[right_idx] / bins[right_idx];
//         float x_diff = right_x - left_x;
//         float gate_angle = atan2(x_diff, detected_width);
        
//         // ===== STEP 5: Validate gate width =====
        
//         if (detected_width < gate_width_min || detected_width > gate_width_max) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Invalid gate width: %.2fm (expected %.2f-%.2fm)",
//                 detected_width, gate_width_min, gate_width_max);
            
//             // Stop lateral movement but maintain altitude with STABILITY FIX
//             if (!test_mode) {
//                 // RANGEFINDER-BASED ALTITUDE CONTROL
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.10f) {  // 10cm deadband
//                     vz_global = 0.3f * altitude_error;
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // No yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 6: Calculate lateral offset =====
        
//         float center_y = (left_y + right_y) / 2.0f;
//         float lateral_error = center_y;  // Positive = gate to the right
        
//         // ===== STEP 7: Check if centered (lateral AND yaw aligned) =====
//         const float yaw_tolerance = 0.087f;  // 5 degrees in radians
//         float yaw_error = normalize_angle(gate_angle);
//         bool is_yaw_aligned = (std::abs(yaw_error) < yaw_tolerance);
        
//         bool is_laterally_centered = (std::abs(lateral_error) < tolerance);
//         bool is_centered = is_laterally_centered && is_yaw_aligned;
        
//         if (is_centered) {
//             consecutive_centered_count++;
            
//             if (test_mode) {
//                 RCLCPP_INFO(node->get_logger(),
//                     "CENTERED! Frame %d/%d | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error);
//             } else {
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "CENTERED! Frame %d/%d | Gate: [L:%.2fm R:%.2fm W:%.2fm] | Err: %.3fm YawErr:%.1f° | Pts[L:%d R:%d]",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI,
//                     bins[left_idx], bins[right_idx]);
//             }
            
//             // Stop lateral movement when centered, but maintain altitude!
//             if (!test_mode) {
//                 // RANGEFINDER-BASED ALTITUDE CONTROL (more stable when centered)
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
                
//                 // Apply low-pass filter
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 // Calculate altitude error
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
                
//                 // Tighter control when centered (smaller deadband)
//                 if (std::abs(altitude_error) > 0.08f) {  // 8cm deadband when centered
//                     vz_global = 0.35f * altitude_error;  // Slightly higher gain for precision
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;  // No forward/backward
//                 stop_cmd.twist.linear.y = 0.0;  // No lateral movement
//                 stop_cmd.twist.linear.z = vz_global;  // Maintain altitude!
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // CRITICAL: Lock yaw when centered
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             if (consecutive_centered_count >= required_centered_frames) {
//                 if (test_mode) {
//                     RCLCPP_INFO(node->get_logger(), "TEST MODE: Centering validation SUCCESSFUL!");
//                 } else {
//                     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESSFUL!");
//                 }
//                 status = true;
//                 break;
//             }

            
//         } else {
//             consecutive_centered_count = 0;
            
//             // ===== STEP 8: Calculate and publish velocity command =====
            
//             // In BODY frame: X=forward, Y=left, Z=up
//             // lateral_error > 0 means gate is to the RIGHT, need to move RIGHT (negative Y in body frame... wait no!)
//             // Actually in Livox/ROS convention:
//             // - Y positive = left side
//             // - Y negative = right side
//             // So if gate center has positive Y, it's on the left, we need to move left (positive Y velocity)
//             // If gate center has negative Y, it's on the right, we need to move right (negative Y velocity)
//             // Simple: velocity should match the error sign (no negative!)
            
//             float target_velocity_body_y = proportional_gain * lateral_error;  // No negative sign!
            
//             // Apply velocity limits
//             target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
            
//             if (test_mode) {
//                 std::string direction = (lateral_error > 0) ? "RIGHT" : "LEFT";
                
//                 RCLCPP_INFO(node->get_logger(),
//                     "NOT CENTERED | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm %s | Vel:%.3fm/s",
//                     left_y, right_y, detected_width, lateral_error, direction.c_str(), target_velocity_body_y);
//             }
            
//             if (!test_mode) {
//                 // Get current pose for frame transformation
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float current_yaw = getHeading(current_pose.pose.orientation);
                
//                 // Transform velocity from BODY frame to GLOBAL frame (map)
//                 // This is critical for correct movement regardless of drone orientation!
//                 // Global frame transformation:
//                 // vx_global = vx_body * cos(yaw) - vy_body * sin(yaw)
//                 // vy_global = vx_body * sin(yaw) + vy_body * cos(yaw)
//                 float vx_body = 0.0;  // No forward/backward during centering
//                 float vy_body = target_velocity_body_y;
                
//                 float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
//                 float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
                
//                 // RANGEFINDER-BASED ALTITUDE CONTROL (active centering mode)
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
                
//                 // Apply low-pass filter
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 // Calculate altitude error
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
                
//                 // Moderate control during active centering (medium deadband)
//                 if (std::abs(altitude_error) > 0.10f) {  // 10cm deadband
//                     vz_global = 0.3f * altitude_error;  // Moderate gain
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 // Yaw control (proportional correction to align with gate)
//                 const float kp_yaw = 0.3f;  // Proportional gain for yaw
//                 const float max_angular_vel = 0.3f;  // rad/s
//                 float angular_z = kp_yaw * yaw_error;
//                 angular_z = std::max(-max_angular_vel, std::min(max_angular_vel, angular_z));

//                 // Publish velocity command in GLOBAL frame (map/odom)
//                 geometry_msgs::msg::TwistStamped twist_cmd;
//                 twist_cmd.header.stamp = node->now();
//                 twist_cmd.header.frame_id = "map";  // Global frame for correct orientation
//                 twist_cmd.twist.linear.x = vx_global;
//                 twist_cmd.twist.linear.y = vy_global;
//                 twist_cmd.twist.linear.z = vz_global;
//                 twist_cmd.twist.angular.x = 0.0;
//                 twist_cmd.twist.angular.y = 0.0;
//                 twist_cmd.twist.angular.z = -angular_z;  // Yaw correction
                
//                 node->publishLocalVelocity(twist_cmd);
                
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "Centering: Gate[L:%.2f R:%.2f W:%.2fm] | LateralErr:%.3fm YawErr:%.1f° | Vel[x:%.3f y:%.3f z:%.3f ω:%.3f] | Pts[L:%d R:%d]",
//                     left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI,
//                     vx_global, vy_global, vz_global, angular_z,
//                     bins[left_idx], bins[right_idx]);
//             }
//         }
        
//         rclcpp::spin_some(node);
//         rate.sleep();
//     }
    
//     // Final stop
//     if (!test_mode) {
//         geometry_msgs::msg::TwistStamped stop_cmd;
//         stop_cmd.header.stamp = node->now();
//         stop_cmd.header.frame_id = "map";
//         stop_cmd.twist.linear.x = 0.0;
//         stop_cmd.twist.linear.y = 0.0;
//         stop_cmd.twist.linear.z = 0.0;
//         stop_cmd.twist.angular.x = 0.0;
//         stop_cmd.twist.angular.y = 0.0;
//         stop_cmd.twist.angular.z = 0.0;
//         node->publishLocalVelocity(stop_cmd);
//     }
    
//     if (status) {
//         if (test_mode) {
//             RCLCPP_INFO(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation COMPLETED ===");
//         } else {
//             RCLCPP_INFO(node->get_logger(), "=== SIMPLE Livox Gate Centering COMPLETED ===");
//         }
//     } else {
//         if (test_mode) {
//             RCLCPP_ERROR(node->get_logger(), "\n=== TEST MODE: LiDAR Centering Validation FAILED ===");
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "=== SIMPLE Livox Gate Centering FAILED ===");
//         }
//     }
// }

// void centeringGateLivoxSimpleRear(
//     const std::shared_ptr<DroneController>&node,
//     rclcpp::Rate &rate,
//     geometry_msgs::msg::PoseStamped &posee,
//     float gate_width,
//     float tolerance,
//     bool &status,
//     float max_velocity,
//     float proportional_gain,
//     float max_time,
//     bool test_mode)
// {
//     if (test_mode) {
//         RCLCPP_INFO(node->get_logger(), "=== LIVOX CENTERING TEST MODE (REAR GATE) ===");
//     } else {
//         RCLCPP_INFO(node->get_logger(), "=== Starting SIMPLE Livox REAR Gate Centering ===");
//     }
//     RCLCPP_INFO(node->get_logger(), "Expected gate width: %.2fm, Tolerance: %.3fm", 
//                 gate_width, tolerance);
//     if (!test_mode) {
//         RCLCPP_INFO(node->get_logger(), "Strategy: Grid binning + peak detection + REAR view (backward movement)");
//     }

//     float detection_range_min = 1.5f;
//     float detection_range_max = 2.5f;
//     float roi_lateral = 2.0f;
//     int min_cluster_points = 10;
    
//     node->get_parameter("gate_centering.detection_range_min", detection_range_min);
//     node->get_parameter("gate_centering.detection_range_max", detection_range_max);
//     node->get_parameter("gate_centering.roi_lateral", roi_lateral);
//     node->get_parameter("gate_centering.min_cluster_points", min_cluster_points);
    
//     // ROI parameters - REAR VIEW (negative X = behind drone in body frame)
//     const float x_min = -detection_range_max;  // Behind drone: -2.5m to -1.5m
//     const float x_max = -detection_range_min;
//     const float y_min = -roi_lateral;
//     const float y_max = roi_lateral;
//     const float z_min = 0.0f;
//     const float z_max = 2.0f;
    
//     // Grid binning parameters
//     const int num_bins = 60;
//     const float bin_size = (y_max - y_min) / num_bins;
//     const int min_points_per_pole = min_cluster_points;
    
//     // Gate validation parameters
//     const float gate_width_min = 0.8f;
//     const float gate_width_max = 2.5f;
    
//     RCLCPP_INFO(node->get_logger(), "ROI: X[%.1f-%.1fm] Y[%.1f-%.1fm] Z[%.1f-%.1fm]",
//                 x_min, x_max, y_min, y_max, z_min, z_max);
//     RCLCPP_INFO(node->get_logger(), "Grid: %d bins @ %.2fm, Min points/pole: %d",
//                 num_bins, bin_size, min_points_per_pole);
    
//     // Subscribe to Livox CustomMsg
//     sensor_msgs::msg::CustomMsg::SharedPtr latest_cloud = nullptr;
    
//     auto lidar_sub = node->create_subscription<sensor_msgs::msg::CustomMsg>(
//         "/livox/lidar", 10,
//         [&latest_cloud](const sensor_msgs::msg::CustomMsg::SharedPtr msg) {
//             latest_cloud = msg;
//         });
    
//     RCLCPP_INFO(node->get_logger(), "Subscribing to Livox CustomMsg on /livox/lidar");

//     // Centering control variables
//     auto start_time = node->now();
//     int consecutive_centered_count = 0;
//     const int required_centered_frames = 10;
//     status = false;
    
//     float initial_z = node->getCurrentLocalPose().pose.position.z;
//     geometry_msgs::msg::PoseStamped hold_pose = node->getCurrentLocalPose();
    
//     while (rclcpp::ok()) {
//         // Check timeout
//         auto elapsed = (node->now() - start_time).seconds();
//         if (elapsed > max_time) {
//             RCLCPP_ERROR(node->get_logger(), "Gate centering timeout after %.1fs", elapsed);
//             break;
//         }
        
//         if (!latest_cloud || latest_cloud->data.empty()) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
//                 "Waiting for Livox CustomMsg data on /livox/lidar...");
            
//             if (!test_mode) {
//                 hold_pose = node->getCurrentLocalPose();
//                 hold_pose.header.stamp = node->now();
//                 node->publishLocalPosition(hold_pose);
//             }
            
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 1: Extract and filter points by ROI using CustomMsg =====
        
//         // Find field offsets for x, y, z in CustomMsg
//         int x_offset = -1, y_offset = -1, z_offset = -1;
//         for (size_t i = 0; i < latest_cloud->fields.size(); ++i) {
//             if (latest_cloud->fields[i].name == "x") x_offset = latest_cloud->fields[i].offset;
//             else if (latest_cloud->fields[i].name == "y") y_offset = latest_cloud->fields[i].offset;
//             else if (latest_cloud->fields[i].name == "z") z_offset = latest_cloud->fields[i].offset;
//         }
        
//         if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
//             RCLCPP_ERROR(node->get_logger(), "Could not find x,y,z fields in CustomMsg");
//             break;
//         }
        
//         // ===== STEP 2: Grid binning - count points per lateral bin =====
        
//         int bins[num_bins] = {0};
//         float bins_x_sum[num_bins] = {0.0f};
//         int total_roi_points = 0;
        
//         const uint8_t* data_ptr = latest_cloud->data.data();
//         size_t point_step = latest_cloud->point_step;
//         size_t num_points = latest_cloud->width * latest_cloud->height;
        
//         for (size_t i = 0; i < num_points; ++i) {
//             const uint8_t* point_data = data_ptr + i * point_step;
            
//             float x, y, z;
//             std::memcpy(&x, point_data + x_offset, sizeof(float));
//             std::memcpy(&y, point_data + y_offset, sizeof(float));
//             std::memcpy(&z, point_data + z_offset, sizeof(float));
            
//             if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
//             if (x >= x_min && x <= x_max &&
//                 y >= y_min && y <= y_max &&
//                 z >= z_min && z <= z_max) {
                
//                 int bin_idx = static_cast<int>((y - y_min) / bin_size);
                
//                 if (bin_idx >= 0 && bin_idx < num_bins) {
//                     bins[bin_idx]++;
//                     bins_x_sum[bin_idx] += x;
//                     total_roi_points++;
//                 }
//             }
//         }
        
//         if (total_roi_points < min_points_per_pole * 2) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Insufficient points in ROI: %d (need %d)", 
//                 total_roi_points, min_points_per_pole * 2);
            
//             if (!test_mode) {
//                 float current_z = node->getCurrentLocalPose().pose.position.z;
//                 float altitude_error = initial_z - current_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 3 (MODIFIED): Find ALL peaks and select RIGHTMOST valid gate =====
        
//         std::vector<int> peak_indices;
        
//         for (int i = 0; i < num_bins; i++) {
//             if (bins[i] >= min_points_per_pole) {
//                 peak_indices.push_back(i);
//             }
//         }
        
//         if (peak_indices.size() < 2) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Insufficient peaks detected: %zu (need at least 2)", 
//                 peak_indices.size());
            
//             if (!test_mode) {
//                 float current_z = node->getCurrentLocalPose().pose.position.z;
//                 float altitude_error = initial_z - current_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 4: Find 2 strongest peaks (standard gate detection for rear view) =====
//         // For rear centering, we use the same logic as forward but will move backward
//         std::sort(peak_indices.begin(), peak_indices.end(),
//             [&bins](int a, int b) { return bins[a] > bins[b]; });
        
//         bool gate_found = false;
//         int left_idx = -1;
//         int right_idx = -1;
//         float detected_width = 0.0f;
        
//         // Try combinations of strongest peaks
//         for (size_t i = 0; i < peak_indices.size() - 1; i++) {
//             for (size_t j = i + 1; j < peak_indices.size(); j++) {
//                 int idx1 = peak_indices[i];
//                 int idx2 = peak_indices[j];
                
//                 // Ensure idx1 < idx2 (idx1 is more right, idx2 is more left)
//                 if (idx1 > idx2) std::swap(idx1, idx2);
                
//                 float y1 = y_min + (idx1 + 0.5f) * bin_size;
//                 float y2 = y_min + (idx2 + 0.5f) * bin_size;
//                 float width = y2 - y1;
                
//                 // Check if valid gate width
//                 if (width >= gate_width_min && width <= gate_width_max) {
//                     right_idx = idx1;  // Right pole
//                     left_idx = idx2;   // Left pole
//                     detected_width = width;
//                     gate_found = true;
//                     break;
//                 }
//             }
//             if (gate_found) break;
//         }
        
//         if (!gate_found) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "REAR: No valid gate pair found among %zu peaks (width must be %.2f-%.2fm)",
//                 peak_indices.size(), gate_width_min, gate_width_max);
            
//             if (!test_mode) {
//                 float current_z = node->getCurrentLocalPose().pose.position.z;
//                 float altitude_error = initial_z - current_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // Calculate positions and angle
//         float left_y = y_min + (left_idx + 0.5f) * bin_size;
//         float right_y = y_min + (right_idx + 0.5f) * bin_size;
        
//         float left_x = bins_x_sum[left_idx] / bins[left_idx];
//         float right_x = bins_x_sum[right_idx] / bins[right_idx];
//         float x_diff = right_x - left_x;
//         float gate_angle = atan2(x_diff, detected_width);
        
//         // ===== STEP 5: Calculate lateral offset =====
        
//         float center_y = (left_y + right_y) / 2.0f;
//         float lateral_error = center_y;
        
//         // ===== STEP 6: Check if centered (lateral AND yaw aligned) =====
//         const float yaw_tolerance = 0.087f;
//         float yaw_error = normalize_angle(gate_angle);
//         bool is_yaw_aligned = (std::abs(yaw_error) < yaw_tolerance);
        
//         bool is_laterally_centered = (std::abs(lateral_error) < tolerance);
//         bool is_centered = is_laterally_centered && is_yaw_aligned;
        
//         if (is_centered) {
//             consecutive_centered_count++;
            
//             if (test_mode) {
//                 RCLCPP_INFO(node->get_logger(),
//                     "REAR CENTERED! Frame %d/%d | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error);
//             } else {
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "REAR CENTERED! Frame %d/%d | Gate: [L:%.2fm R:%.2fm W:%.2fm] | Err: %.3fm YawErr:%.1f° | Pts[L:%d R:%d]",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI,
//                     bins[left_idx], bins[right_idx]);
//             }
            
//             // CRITICAL: STOP all movement when centered to prevent oscillation!
//             if (!test_mode) {
//                 float current_z = node->getCurrentLocalPose().pose.position.z;
//                 float altitude_error = initial_z - current_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;  // STOP forward/backward movement
//                 stop_cmd.twist.linear.y = 0.0;  // STOP lateral movement
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // STOP yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             if (consecutive_centered_count >= required_centered_frames) {
//                 if (test_mode) {
//                     RCLCPP_INFO(node->get_logger(), "TEST MODE: Centering validation SUCCESSFUL!");
//                 } else {
//                     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESSFUL!");
//                 }
//                 status = true;
//                 break;
//             }
//         } else {
//             consecutive_centered_count = 0;
            
//             // ===== STEP 7: Calculate and publish velocity command =====
//             // REAR CENTERING STRATEGY (LATERAL ONLY):
//             // 1. Gate is BEHIND drone (detected in negative X ROI: -2.5m to -1.5m)
//             // 2. Drone stays in place (NO forward/backward movement)
//             // 3. Only adjust lateral position (vy_body) to center on gate
//             // 4. When centered, STOP all movement (handled above in is_centered block)
//             //
//             // Lateral error convention:
//             // - lateral_error > 0: gate center is LEFT of drone (Y+), need to move LEFT (vy+)
//             // - lateral_error < 0: gate center is RIGHT of drone (Y-), need to move RIGHT (vy-)
//             //
//             // Forward/Backward control:
//             // - vx_body = 0: NO forward/backward movement (centering only)
            
//             float target_velocity_body_y = proportional_gain * lateral_error;
//             target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
            
//             if (test_mode) {
//                 std::string gate_position = (lateral_error > 0) ? "LEFT" : "RIGHT";
//                 std::string move_lateral = (target_velocity_body_y > 0) ? "LEFT(+Y)" : "RIGHT(-Y)";
                
//                 RCLCPP_INFO(node->get_logger(),
//                     "REAR CENTERING | Gate[L:%.2fm R:%.2fm W:%.2fm] | Err:%.3fm (%s of drone) | Move: %s(%.3fm/s) ONLY",
//                     left_y, right_y, detected_width, lateral_error, gate_position.c_str(),
//                     move_lateral.c_str(), std::abs(target_velocity_body_y));
//             }
            
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float current_yaw = getHeading(current_pose.pose.orientation);
                
//                 // Body frame velocities:
//                 // - vx_body: ZERO = NO forward/backward movement (lateral centering only)
//                 // - vy_body: proportional to lateral error (+ = left, - = right)
//                 float vx_body = 0.0f;  // NO forward/backward movement
//                 float vy_body = target_velocity_body_y;
                
//                 // Transform from body frame to global frame (map/odom)
//                 float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
//                 float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
                
//                 // Altitude hold with gentle correction
//                 float current_z = node->getCurrentLocalPose().pose.position.z;
//                 float altitude_error = initial_z - current_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.1f) {
//                     vz_global = 0.3f * altitude_error;
//                     vz_global = std::max(-0.2f, std::min(0.2f, vz_global));
//                 }
                
//                 // Yaw control to align with gate
//                 const float kp_yaw = 0.3f;
//                 const float max_angular_vel = 0.3f;
//                 float angular_z = kp_yaw * yaw_error;
//                 angular_z = std::max(-max_angular_vel, std::min(max_angular_vel, angular_z));

//                 geometry_msgs::msg::TwistStamped twist_cmd;
//                 twist_cmd.header.stamp = node->now();
//                 twist_cmd.header.frame_id = "map";
//                 twist_cmd.twist.linear.x = vx_global;
//                 twist_cmd.twist.linear.y = vy_global;
//                 twist_cmd.twist.linear.z = vz_global;
//                 twist_cmd.twist.angular.x = 0.0;
//                 twist_cmd.twist.angular.y = 0.0;
//                 twist_cmd.twist.angular.z = -angular_z;  // Negative for proper rotation direction
                
//                 node->publishLocalVelocity(twist_cmd);
                
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "REAR Centering | Gate[L:%.2f R:%.2f W:%.2fm] | LateralErr:%.3fm YawErr:%.1f° | Body[vx:%.3f(BACK) vy:%.3f] → Global[vx:%.3f vy:%.3f vz:%.3f ω:%.3f] | Pts[L:%d R:%d]",
//                     left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI,
//                     vx_body, vy_body, vx_global, vy_global, vz_global, angular_z,
//                     bins[left_idx], bins[right_idx]);
//             }
//         }
        
//         rclcpp::spin_some(node);
//         rate.sleep();
//     }
    
//     // Final stop
//     if (!test_mode) {
//         geometry_msgs::msg::TwistStamped stop_cmd;
//         stop_cmd.header.stamp = node->now();
//         stop_cmd.header.frame_id = "map";
//         stop_cmd.twist.linear.x = 0.0;
//         stop_cmd.twist.linear.y = 0.0;
//         stop_cmd.twist.linear.z = 0.0;
//         stop_cmd.twist.angular.x = 0.0;
//         stop_cmd.twist.angular.y = 0.0;
//         stop_cmd.twist.angular.z = 0.0;
//         node->publishLocalVelocity(stop_cmd);
//     }
    
//     if (status) {
//         if (test_mode) {
//             RCLCPP_INFO(node->get_logger(), "\n=== TEST MODE: REAR LiDAR Centering COMPLETED ===");
//         } else {
//             RCLCPP_INFO(node->get_logger(), "=== SIMPLE Livox REAR Gate Centering COMPLETED ===");
//         }
//     } else {
//         if (test_mode) {
//             RCLCPP_ERROR(node->get_logger(), "\n=== TEST MODE: REAR LiDAR Centering FAILED ===");
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "=== SIMPLE Livox REAR Gate Centering FAILED ===");
//         }
//     }
// }

// void centeringGateLivoxSimpleRear(
//     const std::shared_ptr<DroneController>&node,
//     rclcpp::Rate &rate,
//     geometry_msgs::msg::PoseStamped &posee,
//     float gate_width,
//     float tolerance,
//     bool &status,
//     float max_velocity,
//     float proportional_gain,
//     float max_time,
//     bool test_mode)
// {
//     if (test_mode) {
//         RCLCPP_INFO(node->get_logger(), "=== LIVOX CENTERING TEST MODE (REAR GATE) ===");
//     } else {
//         RCLCPP_INFO(node->get_logger(), "=== Starting SIMPLE Livox REAR Gate Centering ===");
//     }
//     RCLCPP_INFO(node->get_logger(), "Expected gate width: %.2fm, Tolerance: %.3fm", 
//                 gate_width, tolerance);
//     if (!test_mode) {
//         RCLCPP_INFO(node->get_logger(), "Strategy: Grid binning + peak detection + REAR view (backward movement)");
//     }

//     float detection_range_min = 1.5f;
//     float detection_range_max = 2.5f;
//     float roi_lateral = 2.0f;
//     int min_cluster_points = 10;
    
//     node->get_parameter("gate_centering.detection_range_min", detection_range_min);
//     node->get_parameter("gate_centering.detection_range_max", detection_range_max);
//     node->get_parameter("gate_centering.roi_lateral", roi_lateral);
//     node->get_parameter("gate_centering.min_cluster_points", min_cluster_points);
    
//     // ROI parameters - REAR VIEW (negative X = behind drone in body frame)
//     const float x_min = -detection_range_max;  // Behind drone: -2.5m to -1.5m
//     const float x_max = -detection_range_min;
//     const float y_min = -roi_lateral;
//     const float y_max = roi_lateral;
//     const float z_min = 0.0f;
//     const float z_max = 2.0f;
    
//     // Grid binning parameters
//     const int num_bins = 60;
//     const float bin_size = (y_max - y_min) / num_bins;
//     const int min_points_per_pole = min_cluster_points;
    
//     // Gate validation parameters
//     const float gate_width_min = 0.8f;
//     const float gate_width_max = 2.5f;
    
//     RCLCPP_INFO(node->get_logger(), "ROI: X[%.1f-%.1fm] Y[%.1f-%.1fm] Z[%.1f-%.1fm]",
//                 x_min, x_max, y_min, y_max, z_min, z_max);
//     RCLCPP_INFO(node->get_logger(), "Grid: %d bins @ %.2fm, Min points/pole: %d",
//                 num_bins, bin_size, min_points_per_pole);
    
//     // Subscribe to Livox Custom LiDAR data
//     livox_ros_driver2::msg::CustomMsg::SharedPtr latest_cloud = nullptr;
    
//     auto lidar_sub = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10,
//         [&latest_cloud](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//             latest_cloud = msg;
//         });
    
//     RCLCPP_INFO(node->get_logger(), "Subscribing to Livox CustomMsg on /livox/lidar");

//     // Centering control variables
//     auto start_time = node->now();
//     int consecutive_centered_count = 0;
//     const int required_centered_frames = 1;
//     status = false;
    
//     // Get initial altitude for stability (using relative altitude)
//     float initial_z = node->getRelativeAltitude();
    
//     // PX4 CRITICAL: Prepare hold position for streaming when waiting
//     geometry_msgs::msg::PoseStamped hold_pose = node->getCurrentLocalPose();
    
//     while (rclcpp::ok()) {
//         // Check timeout
//         auto elapsed = (node->now() - start_time).seconds();
//         if (elapsed > max_time) {
//             RCLCPP_ERROR(node->get_logger(), "Gate centering timeout after %.1fs", elapsed);
//             break;
//         }
        
//         if (!latest_cloud || latest_cloud->point_num == 0) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
//                 "Waiting for Livox CustomMsg data on /livox/lidar...");
            
//             // PX4 CRITICAL: MUST stream setpoint even when waiting for data!
//             if (!test_mode) {
//                 hold_pose = node->getCurrentLocalPose();
//                 hold_pose.header.stamp = node->now();
//                 node->publishLocalPosition(hold_pose);
//             }
            
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 1: Extract and filter points by ROI using CustomMsg =====
        
//         // CustomMsg structure:
//         // - header: std_msgs/Header
//         // - timebase: uint64
//         // - point_num: uint32 (total number of points)
//         // - lidar_id: uint8
//         // - rsvd: uint8[3]
//         // - points: CustomPoint[] array
//         //
//         // CustomPoint structure:
//         // - offset_time: uint32
//         // - x, y, z: float32
//         // - reflectivity: uint8
//         // - tag: uint8
//         // - line: uint8
        
//         // ===== STEP 2: Grid binning - count points per lateral bin =====
        
//         int bins[num_bins] = {0};
//         float bins_x_sum[num_bins] = {0.0f};
//         int total_roi_points = 0;
        
//         // Access points directly from CustomMsg
//         const auto& points = latest_cloud->points;
//         uint32_t num_points = latest_cloud->point_num;
        
//         // Validate point count
//         if (num_points != static_cast<uint32_t>(points.size())) {
//             RCLCPP_WARN(node->get_logger(), 
//                 "CustomMsg point_num (%u) doesn't match points array size (%zu)", 
//                 latest_cloud->point_num, points.size());
//             num_points = std::min(num_points, static_cast<uint32_t>(points.size()));
//         }
        
//         for (size_t i = 0; i < num_points; ++i) {
//             const auto& point = points[i];
            
//             float x = point.x;
//             float y = point.y;
//             float z = point.z;
            
//             if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
//             if (x >= x_min && x <= x_max &&
//                 y >= y_min && y <= y_max &&
//                 z >= z_min && z <= z_max) {
                
//                 int bin_idx = static_cast<int>((y - y_min) / bin_size);
                
//                 if (bin_idx >= 0 && bin_idx < num_bins) {
//                     bins[bin_idx]++;
//                     bins_x_sum[bin_idx] += x;
//                     total_roi_points++;
//                 }
//             }
//         }
        
//         if (total_roi_points < min_points_per_pole * 2) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Insufficient points in ROI: %d (need %d)", 
//                 total_roi_points, min_points_per_pole * 2);
            
//             // Stop lateral movement but maintain altitude (using relative altitude)
//             if (!test_mode) {
//                 float current_relative_z = node->getRelativeAltitude();
//                 float altitude_error = initial_z - current_relative_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 3 (MODIFIED): Find ALL peaks and select RIGHTMOST valid gate =====
        
//         std::vector<int> peak_indices;
        
//         for (int i = 0; i < num_bins; i++) {
//             if (bins[i] >= min_points_per_pole) {
//                 peak_indices.push_back(i);
//             }
//         }
        
//         if (peak_indices.size() < 2) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Insufficient peaks detected: %zu (need at least 2)", 
//                 peak_indices.size());
            
//             if (!test_mode) {
//                 float current_relative_z = node->getRelativeAltitude();
//                 float altitude_error = initial_z - current_relative_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 4: Find 2 strongest peaks (standard gate detection for rear view) =====
//         // For rear centering, we use the same logic as forward but will move backward
//         std::sort(peak_indices.begin(), peak_indices.end(),
//             [&bins](int a, int b) { return bins[a] > bins[b]; });
        
//         bool gate_found = false;
//         int left_idx = -1;
//         int right_idx = -1;
//         float detected_width = 0.0f;
        
//         // Try combinations of strongest peaks
//         for (size_t i = 0; i < peak_indices.size() - 1; i++) {
//             for (size_t j = i + 1; j < peak_indices.size(); j++) {
//                 int idx1 = peak_indices[i];
//                 int idx2 = peak_indices[j];
                
//                 // Ensure idx1 < idx2 (idx1 is more right, idx2 is more left)
//                 if (idx1 > idx2) std::swap(idx1, idx2);
                
//                 float y1 = y_min + (idx1 + 0.5f) * bin_size;
//                 float y2 = y_min + (idx2 + 0.5f) * bin_size;
//                 float width = y2 - y1;
                
//                 // Check if valid gate width
//                 if (width >= gate_width_min && width <= gate_width_max) {
//                     right_idx = idx1;  // Right pole
//                     left_idx = idx2;   // Left pole
//                     detected_width = width;
//                     gate_found = true;
//                     break;
//                 }
//             }
//             if (gate_found) break;
//         }
        
//         if (!gate_found) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "REAR: No valid gate pair found among %zu peaks (width must be %.2f-%.2fm)",
//                 peak_indices.size(), gate_width_min, gate_width_max);
            
//             if (!test_mode) {
//                 float current_relative_z = node->getRelativeAltitude();
//                 float altitude_error = initial_z - current_relative_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // Calculate positions and angle
//         float left_y = y_min + (left_idx + 0.5f) * bin_size;
//         float right_y = y_min + (right_idx + 0.5f) * bin_size;
        
//         float left_x = bins_x_sum[left_idx] / bins[left_idx];
//         float right_x = bins_x_sum[right_idx] / bins[right_idx];
//         float x_diff = right_x - left_x;
//         float gate_angle = atan2(x_diff, detected_width);
        
//         // ===== STEP 5: Calculate lateral offset =====
        
//         float center_y = (left_y + right_y) / 2.0f;
//         float lateral_error = center_y;
        
//         // ===== STEP 6: Check if centered (lateral AND yaw aligned) =====
//         const float yaw_tolerance = 0.087f;
//         float yaw_error = normalize_angle(gate_angle);
//         bool is_yaw_aligned = (std::abs(yaw_error) < yaw_tolerance);
        
//         bool is_laterally_centered = (std::abs(lateral_error) < tolerance);
//         bool is_centered = is_laterally_centered && is_yaw_aligned;
        
//         if (is_centered) {
//             consecutive_centered_count++;
            
//             if (test_mode) {
//                 RCLCPP_INFO(node->get_logger(),
//                     "REAR CENTERED! Frame %d/%d | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error);
//             } else {
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "REAR CENTERED! Frame %d/%d | Gate: [L:%.2fm R:%.2fm W:%.2fm] | Err: %.3fm YawErr:%.1f° | Pts[L:%d R:%d]",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI,
//                     bins[left_idx], bins[right_idx]);
//             }
            
//             // CRITICAL: STOP all movement when centered to prevent oscillation!
//             if (!test_mode) {
//                 float current_relative_z = node->getRelativeAltitude();
//                 float altitude_error = initial_z - current_relative_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.05f) {
//                     vz_global = 0.4f * altitude_error;
//                     vz_global = std::max(-0.3f, std::min(0.3f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;  // STOP forward/backward movement
//                 stop_cmd.twist.linear.y = 0.0;  // STOP lateral movement
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // STOP yaw rotation
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             if (consecutive_centered_count >= required_centered_frames) {
//                 if (test_mode) {
//                     RCLCPP_INFO(node->get_logger(), "TEST MODE: Centering validation SUCCESSFUL!");
//                 } else {
//                     RCLCPP_INFO(node->get_logger(), "Gate centering SUCCESSFUL!");
//                 }
//                 status = true;
//                 break;
//             }
//         } else {
//             consecutive_centered_count = 0;
            
//             // ===== STEP 7: Calculate and publish velocity command =====
//             // REAR CENTERING STRATEGY (LATERAL ONLY):
//             // 1. Gate is BEHIND drone (detected in negative X ROI: -2.5m to -1.5m)
//             // 2. Drone stays in place (NO forward/backward movement)
//             // 3. Only adjust lateral position (vy_body) to center on gate
//             // 4. When centered, STOP all movement (handled above in is_centered block)
//             //
//             // Lateral error convention:
//             // - lateral_error > 0: gate center is LEFT of drone (Y+), need to move LEFT (vy+)
//             // - lateral_error < 0: gate center is RIGHT of drone (Y-), need to move RIGHT (vy-)
//             //
//             // Forward/Backward control:
//             // - vx_body = 0: NO forward/backward movement (centering only)
            
//             float target_velocity_body_y = proportional_gain * lateral_error;
//             target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
            
//             if (test_mode) {
//                 std::string gate_position = (lateral_error > 0) ? "LEFT" : "RIGHT";
//                 std::string move_lateral = (target_velocity_body_y > 0) ? "LEFT(+Y)" : "RIGHT(-Y)";
                
//                 RCLCPP_INFO(node->get_logger(),
//                     "REAR CENTERING | Gate[L:%.2fm R:%.2fm W:%.2fm] | Err:%.3fm (%s of drone) | Move: %s(%.3fm/s) ONLY",
//                     left_y, right_y, detected_width, lateral_error, gate_position.c_str(),
//                     move_lateral.c_str(), std::abs(target_velocity_body_y));
//             }
            
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float current_yaw = getHeading(current_pose.pose.orientation);
                
//                 // Body frame velocities:
//                 // - vx_body: ZERO = NO forward/backward movement (lateral centering only)
//                 // - vy_body: proportional to lateral error (+ = left, - = right)
//                 float vx_body = 0.0f;  // NO forward/backward movement
//                 float vy_body = target_velocity_body_y;
                
//                 // Transform from body frame to global frame (map/odom)
//                 float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
//                 float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
                
//                 // Altitude hold with gentle correction
//                 float current_relative_z = node->getRelativeAltitude();
//                 float altitude_error = initial_z - current_relative_z;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.1f) {
//                     vz_global = 0.3f * altitude_error;
//                     vz_global = std::max(-0.2f, std::min(0.2f, vz_global));
//                 }
                
//                 // Yaw control to align with gate
//                 const float kp_yaw = 0.3f;
//                 const float max_angular_vel = 0.3f;
//                 float angular_z = kp_yaw * yaw_error;
//                 angular_z = std::max(-max_angular_vel, std::min(max_angular_vel, angular_z));

//                 geometry_msgs::msg::TwistStamped twist_cmd;
//                 twist_cmd.header.stamp = node->now();
//                 twist_cmd.header.frame_id = "map";
//                 twist_cmd.twist.linear.x = vx_global;
//                 twist_cmd.twist.linear.y = vy_global;
//                 twist_cmd.twist.linear.z = vz_global;
//                 twist_cmd.twist.angular.x = 0.0;
//                 twist_cmd.twist.angular.y = 0.0;
//                 twist_cmd.twist.angular.z = angular_z;  // Negative for proper rotation direction
                
//                 node->publishLocalVelocity(twist_cmd);
                
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "REAR Centering | Gate[L:%.2f R:%.2f W:%.2fm] | LateralErr:%.3fm YawErr:%.1f° | Body[vx:%.3f(BACK) vy:%.3f] → Global[vx:%.3f vy:%.3f vz:%.3f ω:%.3f] | Pts[L:%d R:%d]",
//                     left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI,
//                     vx_body, vy_body, vx_global, vy_global, vz_global, angular_z,
//                     bins[left_idx], bins[right_idx]);
//             }
//         }
        
//         rclcpp::spin_some(node);
//         rate.sleep();
//     }
    
//     // Final stop
//     if (!test_mode) {
//         geometry_msgs::msg::TwistStamped stop_cmd;
//         stop_cmd.header.stamp = node->now();
//         stop_cmd.header.frame_id = "map";
//         stop_cmd.twist.linear.x = 0.0;
//         stop_cmd.twist.linear.y = 0.0;
//         stop_cmd.twist.linear.z = 0.0;
//         stop_cmd.twist.angular.x = 0.0;
//         stop_cmd.twist.angular.y = 0.0;
//         stop_cmd.twist.angular.z = 0.0;
//         node->publishLocalVelocity(stop_cmd);
//     }
    
//     if (status) {
//         if (test_mode) {
//             RCLCPP_INFO(node->get_logger(), "\n=== TEST MODE: REAR LiDAR Centering COMPLETED ===");
//         } else {
//             RCLCPP_INFO(node->get_logger(), "=== SIMPLE Livox REAR Gate Centering COMPLETED ===");
//         }
//     } else {
//         if (test_mode) {
//             RCLCPP_ERROR(node->get_logger(), "\n=== TEST MODE: REAR LiDAR Centering FAILED ===");
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "=== SIMPLE Livox REAR Gate Centering FAILED ===");
//         }
//     }
// }

// void centeringGateLivoxFast(
//     const std::shared_ptr<DroneController>&node,
//     rclcpp::Rate &rate,
//     geometry_msgs::msg::PoseStamped &posee,
//     float gate_width,
//     float tolerance,
//     bool &status,
//     float max_velocity,
//     float proportional_gain,
//     float max_time,
//     bool test_mode)
// {
//     if (test_mode) {
//         RCLCPP_INFO(node->get_logger(), "=== LIVOX FAST CENTERING TEST MODE ===");
//     } else {
//         RCLCPP_INFO(node->get_logger(), "=== Starting FAST Livox Gate Centering ===");
//     }
//     RCLCPP_INFO(node->get_logger(), "Expected gate width: %.2fm, Tolerance: %.3fm", 
//                 gate_width, tolerance);
//     RCLCPP_INFO(node->get_logger(), "Strategy: Direct center calculation + immediate velocity command");

//     float detection_range_min = 1.5f;
//     float detection_range_max = 2.5f;
//     float roi_lateral = 2.0f;
//     int min_cluster_points = 10;
    
//     node->get_parameter("gate_centering.detection_range_min", detection_range_min);
//     node->get_parameter("gate_centering.detection_range_max", detection_range_max);
//     node->get_parameter("gate_centering.roi_lateral", roi_lateral);
//     node->get_parameter("gate_centering.min_cluster_points", min_cluster_points);
    
//     // ROI parameters
//     const float x_min = detection_range_min;
//     const float x_max = detection_range_max;
//     const float y_min = -roi_lateral;
//     const float y_max = roi_lateral;
//     const float z_min = 0.0f;
//     const float z_max = 2.0f;
    
//     // Grid binning parameters
//     const int num_bins = 60;
//     const float bin_size = (y_max - y_min) / num_bins;
//     const int min_points_per_pole = min_cluster_points;
    
//     // Gate validation parameters
//     const float gate_width_min = 0.8f;
//     const float gate_width_max = 2.5f;
    
//     // Fast centering parameters
//     const float fast_velocity_gain = 2.0f;  // Higher gain for faster movement
//     const float approach_distance_threshold = 0.3f;  // Switch to slow mode when closer
    
//     RCLCPP_INFO(node->get_logger(), "ROI: X[%.1f-%.1fm] Y[%.1f-%.1fm] Z[%.1f-%.1fm]",
//                 x_min, x_max, y_min, y_max, z_min, z_max);
//     RCLCPP_INFO(node->get_logger(), "Grid: %d bins @ %.2fm, Min points/pole: %d",
//                 num_bins, bin_size, min_points_per_pole);
//     RCLCPP_INFO(node->get_logger(), "Fast mode: velocity gain=%.1f, approach threshold=%.2fm",
//                 fast_velocity_gain, approach_distance_threshold);
    
//     // Subscribe to Livox Custom LiDAR data
//     livox_ros_driver2::msg::CustomMsg::SharedPtr latest_cloud = nullptr;
    
//     auto lidar_sub = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10,
//         [&latest_cloud](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//             latest_cloud = msg;
//         });
    
//     RCLCPP_INFO(node->get_logger(), "Subscribing to Livox CustomMsg on /livox/lidar");

//     // Centering control variables
//     auto start_time = node->now();
//     int consecutive_centered_count = 0;
//     const int required_centered_frames = 1; // 0.3 second at 10Hz - FAST EXIT when centered!
//     status = false;
    
//     // ALTITUDE STABILITY IMPROVEMENT: Use rangefinder directly (more stable than local pose Z)
//     sensor_msgs::msg::Range initial_rangefinder = node->getCurrentRangefinder();
//     float initial_rangefinder_distance = initial_rangefinder.range;
    
//     // Low-pass filter for rangefinder (reduce noise from surface texture)
//     float filtered_rangefinder = initial_rangefinder_distance;
//     const float rangefinder_filter_alpha = 0.4f;  // Slightly more responsive for rangefinder
    
//     // Reference altitude tracking (to compensate drift over time)
//     float reference_altitude = initial_rangefinder_distance;
//     auto last_reference_update = node->now();
//     const double reference_update_interval = 5.0;  // Update reference every 5 seconds when stable
//     int stable_altitude_count = 0;
//     const int required_stable_frames = 20;  // 2 seconds of stability before updating reference
    
//     RCLCPP_INFO(node->get_logger(), "Initial rangefinder altitude: %.3fm", initial_rangefinder_distance);
    
//     // PX4 CRITICAL: Prepare hold position for streaming when waiting
//     geometry_msgs::msg::PoseStamped hold_pose = node->getCurrentLocalPose();
    
//     while (rclcpp::ok()) {
//         // Check timeout
//         auto elapsed = (node->now() - start_time).seconds();
//         if (elapsed > max_time) {
//             RCLCPP_ERROR(node->get_logger(), "Fast gate centering timeout after %.1fs", elapsed);
//             break; // sekarang coba buat centering livox simple saat sudah center stop proses centering dan lanjutkan. masalahnya kadang log sudah cetak center namun drone masih centering
//         }
        
//         if (!latest_cloud || latest_cloud->point_num == 0) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
//                 "Waiting for Livox CustomMsg data on /livox/lidar...");
            
//             // PX4 CRITICAL: MUST stream setpoint even when waiting for data!
//             if (!test_mode) {
//                 hold_pose = node->getCurrentLocalPose();
//                 hold_pose.header.stamp = node->now();
//                 node->publishLocalPosition(hold_pose);
//             }
            
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 1: Extract and filter points by ROI using CustomMsg =====
        
//         // ===== STEP 2: Grid binning - count points per lateral bin =====
        
//         int bins[num_bins] = {0};
//         float bins_x_sum[num_bins] = {0.0f};
//         int total_roi_points = 0;
        
//         // Access points directly from CustomMsg
//         const auto& points = latest_cloud->points;
//         uint32_t num_points = latest_cloud->point_num;
        
//         // Validate point count
//         if (num_points != static_cast<uint32_t>(points.size())) {
//             RCLCPP_WARN(node->get_logger(), 
//                 "CustomMsg point_num (%u) doesn't match points array size (%zu)", 
//                 latest_cloud->point_num, points.size());
//             num_points = std::min(num_points, static_cast<uint32_t>(points.size()));
//         }
        
//         for (size_t i = 0; i < num_points; ++i) {
//             const auto& point = points[i];
            
//             float x = point.x;
//             float y = point.y;
//             float z = point.z;
            
//             // Skip invalid points
//             if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
//             // Filter by ROI
//             if (x >= x_min && x <= x_max &&
//                 y >= y_min && y <= y_max &&
//                 z >= z_min && z <= z_max) {
                
//                 // Calculate bin index
//                 int bin_idx = static_cast<int>((y - y_min) / bin_size);
                
//                 // Clamp to valid range
//                 if (bin_idx >= 0 && bin_idx < num_bins) {
//                     bins[bin_idx]++;
//                     bins_x_sum[bin_idx] += x;  // Accumulate X for angle calculation
//                     total_roi_points++;
//                 }
//             }
//         }
        
//         if (total_roi_points < min_points_per_pole * 2) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Insufficient points in ROI: %d (need %d)", 
//                 total_roi_points, min_points_per_pole * 2);
            
//             // Stop lateral movement but maintain altitude with RANGEFINDER
//             if (!test_mode) {
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
                
//                 // Apply low-pass filter
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
                
//                 if (std::abs(altitude_error) > 0.10f) {  // 10cm deadband
//                     vz_global = 0.3f * altitude_error;
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 3: Find 2 peak bins (left & right poles) =====
        
//         // Find first peak (highest bin count)
//         int first_peak_idx = -1;
//         int first_peak_count = 0;
        
//         for (int i = 0; i < num_bins; i++) {
//             if (bins[i] > first_peak_count) {
//                 first_peak_count = bins[i];
//                 first_peak_idx = i;
//             }
//         }
        
//         if (first_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "First peak too weak: %d points (need %d)", 
//                 first_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude with RANGEFINDER
//             if (!test_mode) {
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.10f) {
//                     vz_global = 0.3f * altitude_error;
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // Find second peak (must be at least 10 bins away from first peak)
//         int second_peak_idx = -1;
//         int second_peak_count = 0;
//         const int min_peak_separation = 10;  // ~1.0m minimum separation
        
//         for (int i = 0; i < num_bins; i++) {
//             // Must be far enough from first peak
//             if (std::abs(i - first_peak_idx) < min_peak_separation) continue;
            
//             if (bins[i] > second_peak_count) {
//                 second_peak_count = bins[i];
//                 second_peak_idx = i;
//             }
//         }
        
//         if (second_peak_count < min_points_per_pole) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Second peak too weak: %d points (need %d)", 
//                 second_peak_count, min_points_per_pole);
            
//             // Stop lateral movement but maintain altitude with RANGEFINDER
//             if (!test_mode) {
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.10f) {
//                     vz_global = 0.3f * altitude_error;
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 4: Determine left & right poles =====
        
//         int left_idx = (first_peak_idx < second_peak_idx) ? first_peak_idx : second_peak_idx;
//         int right_idx = (first_peak_idx < second_peak_idx) ? second_peak_idx : first_peak_idx;
        
//         // ===== STEP 4: Calculate gate center and angle =====
        
//         float left_y = y_min + (left_idx + 0.5f) * bin_size;
//         float right_y = y_min + (right_idx + 0.5f) * bin_size;
//         float detected_width = right_y - left_y;
        
//         float left_x = bins_x_sum[left_idx] / bins[left_idx];
//         float right_x = bins_x_sum[right_idx] / bins[right_idx];
//         float x_diff = right_x - left_x;
//         float gate_angle = atan2(x_diff, detected_width);
        
//         // ===== STEP 5: Validate gate width =====
        
//         if (detected_width < gate_width_min || detected_width > gate_width_max) {
//             RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//                 "Invalid gate width: %.2fm (expected %.2f-%.2fm)",
//                 detected_width, gate_width_min, gate_width_max);
            
//             // Stop lateral movement but maintain altitude with RANGEFINDER
//             if (!test_mode) {
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
//                 if (std::abs(altitude_error) > 0.10f) {
//                     vz_global = 0.3f * altitude_error;
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;
//                 stop_cmd.twist.linear.y = 0.0;
//                 stop_cmd.twist.linear.z = vz_global;
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             consecutive_centered_count = 0;
//             rclcpp::spin_some(node);
//             rate.sleep();
//             continue;
//         }
        
//         // ===== STEP 6: Calculate center position and error =====
        
//         float center_y = (left_y + right_y) / 2.0f;
//         float lateral_error = center_y;
        
//         // ===== STEP 7: Check if centered =====
        
//         const float yaw_tolerance = 0.087f;  // 5 degrees
//         float yaw_error = normalize_angle(gate_angle);
//         bool is_yaw_aligned = (std::abs(yaw_error) < yaw_tolerance);
//         bool is_laterally_centered = (std::abs(lateral_error) < tolerance);
//         bool is_centered = is_laterally_centered && is_yaw_aligned;
        
//         if (is_centered) {
//             consecutive_centered_count++;
            
//             if (test_mode) {
//                 RCLCPP_INFO(node->get_logger(),
//                     "FAST CENTERED! Frame %d/%d | Gate[L:%.2fm R:%.2fm W:%.2fm] | Offset:%.3fm",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error);
//             } else {
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "FAST CENTERED! Frame %d/%d | Gate: [L:%.2fm R:%.2fm W:%.2fm] | Err: %.3fm YawErr:%.1f°",
//                     consecutive_centered_count, required_centered_frames,
//                     left_y, right_y, detected_width, lateral_error, yaw_error * 180.0 / M_PI);
//             }
            
//             // Stop lateral movement when centered, but maintain altitude with RANGEFINDER!
//             if (!test_mode) {
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
                
//                 // Apply low-pass filter
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 // Calculate altitude error
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
                
//                 // Tighter control when centered (smaller deadband)
//                 if (std::abs(altitude_error) > 0.08f) {  // 8cm deadband when centered
//                     vz_global = 0.35f * altitude_error;  // Slightly higher gain for precision
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 geometry_msgs::msg::TwistStamped stop_cmd;
//                 stop_cmd.header.stamp = node->now();
//                 stop_cmd.header.frame_id = "map";
//                 stop_cmd.twist.linear.x = 0.0;  // No forward/backward
//                 stop_cmd.twist.linear.y = 0.0;  // No lateral movement
//                 stop_cmd.twist.linear.z = vz_global;  // Maintain altitude!
//                 stop_cmd.twist.angular.x = 0.0;
//                 stop_cmd.twist.angular.y = 0.0;
//                 stop_cmd.twist.angular.z = 0.0;  // CRITICAL: Lock yaw when centered
//                 node->publishLocalVelocity(stop_cmd);
//             }
            
//             if (consecutive_centered_count >= required_centered_frames) {
//                 if (test_mode) {
//                     RCLCPP_INFO(node->get_logger(), "TEST MODE: FAST Centering validation SUCCESSFUL!");
//                 } else {
//                     RCLCPP_INFO(node->get_logger(), "FAST Gate centering SUCCESSFUL!");
//                 }
//                 status = true;
//                 break;
//             }
//         } else {
//             consecutive_centered_count = 0;
            
//             // ===== STEP 8: FAST MODE - Direct velocity to center =====
            
//             if (!test_mode) {
//                 geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
//                 float current_yaw = getHeading(current_pose.pose.orientation);
                
//                 // Determine velocity gain based on distance to center
//                 float distance_to_center = std::abs(lateral_error);
//                 float velocity_gain;
                
//                 if (distance_to_center > approach_distance_threshold) {
//                     // FAR FROM CENTER: Use high gain for fast approach
//                     velocity_gain = fast_velocity_gain;
//                 } else {
//                     // CLOSE TO CENTER: Switch to proportional control for smooth landing
//                     velocity_gain = proportional_gain;
//                 }
                
//                 // Calculate target velocity with dynamic gain
//                 float target_velocity_body_y = velocity_gain * lateral_error;
//                 target_velocity_body_y = std::max(-max_velocity, std::min(max_velocity, target_velocity_body_y));
                
//                 // Transform to global frame
//                 float vx_body = 0.0;
//                 float vy_body = target_velocity_body_y;
                
//                 float vx_global = vx_body * cos(current_yaw) - vy_body * sin(current_yaw);
//                 float vy_global = vx_body * sin(current_yaw) + vy_body * cos(current_yaw);
                
//                 // RANGEFINDER-BASED ALTITUDE CONTROL (active centering mode)
//                 sensor_msgs::msg::Range current_rangefinder = node->getCurrentRangefinder();
//                 float current_rangefinder_distance = current_rangefinder.range;
                
//                 // Apply low-pass filter
//                 filtered_rangefinder = rangefinder_filter_alpha * current_rangefinder_distance + 
//                                       (1.0f - rangefinder_filter_alpha) * filtered_rangefinder;
                
//                 // Calculate altitude error
//                 float altitude_error = reference_altitude - filtered_rangefinder;
//                 float vz_global = 0.0;
                
//                 // Moderate control during active centering (medium deadband)
//                 if (std::abs(altitude_error) > 0.10f) {  // 10cm deadband
//                     vz_global = 0.3f * altitude_error;  // Moderate gain
//                     vz_global = std::max(-0.15f, std::min(0.15f, vz_global));
//                 }
                
//                 // Yaw control
//                 const float kp_yaw = 0.3f;
//                 const float max_angular_vel = 0.3f;
//                 float angular_z = kp_yaw * yaw_error;
//                 angular_z = std::max(-max_angular_vel, std::min(max_angular_vel, angular_z));

//                 // Publish velocity command
//                 geometry_msgs::msg::TwistStamped twist_cmd;
//                 twist_cmd.header.stamp = node->now();
//                 twist_cmd.header.frame_id = "map";
//                 twist_cmd.twist.linear.x = vx_global;
//                 twist_cmd.twist.linear.y = vy_global;
//                 twist_cmd.twist.linear.z = vz_global;
//                 twist_cmd.twist.angular.x = 0.0;
//                 twist_cmd.twist.angular.y = 0.0;
//                 twist_cmd.twist.angular.z = -angular_z;
                
//                 node->publishLocalVelocity(twist_cmd);
                
//                 std::string mode = (distance_to_center > approach_distance_threshold) ? "FAST" : "SLOW";
//                 RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
//                     "FAST[%s] | Gate[L:%.2f R:%.2f W:%.2fm] | Err:%.3fm(%.1fx) YawErr:%.1f° | Vel[x:%.3f y:%.3f z:%.3f ω:%.3f]",
//                     mode.c_str(), left_y, right_y, detected_width, 
//                     lateral_error, velocity_gain, yaw_error * 180.0 / M_PI,
//                     vx_global, vy_global, vz_global, angular_z);
//             }
//         }
        
//         rclcpp::spin_some(node);
//         rate.sleep();
//     }
    
//     // Final stop
//     if (!test_mode) {
//         geometry_msgs::msg::TwistStamped stop_cmd;
//         stop_cmd.header.stamp = node->now();
//         stop_cmd.header.frame_id = "map";
//         stop_cmd.twist.linear.x = 0.0;
//         stop_cmd.twist.linear.y = 0.0;
//         stop_cmd.twist.linear.z = 0.0;
//         stop_cmd.twist.angular.x = 0.0;
//         stop_cmd.twist.angular.y = 0.0;
//         stop_cmd.twist.angular.z = 0.0;
//         node->publishLocalVelocity(stop_cmd);
//     }
    
//     if (status) {
//         if (test_mode) {
//             RCLCPP_INFO(node->get_logger(), "\n=== TEST MODE: FAST LiDAR Centering COMPLETED ===");
//         } else {
//             RCLCPP_INFO(node->get_logger(), "=== FAST Livox Gate Centering COMPLETED ===");
//         }
//     } else {
//         if (test_mode) {
//             RCLCPP_ERROR(node->get_logger(), "\n=== TEST MODE: FAST LiDAR Centering FAILED ===");
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "=== FAST Livox Gate Centering FAILED ===");
//         }
//     }
// }

// void followPath(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, const std::vector<PathWaypoint>& waypoints, float speed, float tolerance, bool allow_centering_payload, bool allow_centering_ember, bool disable_z_lock, bool stabilize_at_waypoint, float stabilize_duration) {
//     /**
//      * Follow a path defined by a series of waypoints
//      * Moves the drone through each waypoint sequentially using moveToPoint()
//      */
    
//     if (waypoints.empty()) {
//         RCLCPP_WARN(node->get_logger(), "Path following: waypoints vector is empty!");
//         return;
//     }
    
//     RCLCPP_INFO(node->get_logger(), "=== Starting Path Following ===");
//     RCLCPP_INFO(node->get_logger(), "Total waypoints: %zu", waypoints.size());
//     RCLCPP_INFO(node->get_logger(), "Speed: %.2f m/s | Tolerance: %.2f m", speed, tolerance);
    
//     for (size_t i = 0; i < waypoints.size(); i++) {
//         const PathWaypoint& wp = waypoints[i];
        
//         RCLCPP_INFO(node->get_logger(), "\n>>> Moving to Waypoint %zu/%zu <<<", i+1, waypoints.size());
//         RCLCPP_INFO(node->get_logger(), "    Target: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f rad (%.1f deg)", 
//                     wp.x, wp.y, wp.z, wp.yaw, wp.yaw * RAD2DEG);
        
//         // Create target pose
//         geometry_msgs::msg::Pose target;
//         target.position.x = wp.x;
//         target.position.y = wp.y;
//         target.position.z = wp.z;
//         setHeading(target.orientation, wp.yaw);
        
//         // Move to waypoint
//         moveToPoint(node, rate, posee, target, speed, tolerance, allow_centering_payload, allow_centering_ember, disable_z_lock);
        
//         RCLCPP_INFO(node->get_logger(), "    Waypoint %zu reached!", i+1);
        
//         // Stabilize at waypoint if enabled
//         if (stabilize_at_waypoint && i < waypoints.size() - 1) {  // Don't stabilize at last waypoint
//             RCLCPP_INFO(node->get_logger(), "    Stabilizing for %.1f seconds...", stabilize_duration);
//             stabilize(node, rate, posee, stabilize_duration);
//         }
//     }
    
//     RCLCPP_INFO(node->get_logger(), "=== Path Following Complete ===");
//     RCLCPP_INFO(node->get_logger(), "Final position: x=%.2f, y=%.2f, z=%.2f", 
//                 posee.pose.position.x, posee.pose.position.y, posee.pose.position.z);
// }

// std::vector<PathWaypoint> smoothPathBSpline(const std::vector<PathWaypoint>& waypoints, float smoothness, int degree, int num_points) {
//     /**
//      * Smooth path using B-Spline interpolation
//      * Simplified implementation (not exact scipy.splprep, but similar effect)
//      * Using Catmull-Rom spline for simplicity (similar smoothness to B-spline)
//      */
    
//     if (waypoints.size() < 2) {
//         return waypoints;
//     }
    
//     if (waypoints.size() < 4) {
//         degree = std::min(degree, (int)waypoints.size() - 1);
//     }
    
//     std::vector<PathWaypoint> smooth_path;
    
//     // Generate num_points interpolated waypoints
//     for (int i = 0; i < num_points; i++) {
//         float t_global = (float)i / (num_points - 1);
//         float t_scaled = t_global * (waypoints.size() - 1);
//         int segment = std::min((int)t_scaled, (int)waypoints.size() - 2);
//         float t_local = t_scaled - segment;
        
//         // Get control points (with boundary handling)
//         PathWaypoint p0 = (segment > 0) ? waypoints[segment - 1] : waypoints[segment];
//         PathWaypoint p1 = waypoints[segment];
//         PathWaypoint p2 = waypoints[segment + 1];
//         PathWaypoint p3 = (segment + 2 < waypoints.size()) ? waypoints[segment + 2] : waypoints[segment + 1];
        
//         // Catmull-Rom interpolation (smooth curve through all points)
//         float t2 = t_local * t_local;
//         float t3 = t2 * t_local;
        
//         PathWaypoint interpolated;
//         interpolated.x = 0.5f * ((2.0f * p1.x) +
//                                 (-p0.x + p2.x) * t_local +
//                                 (2.0f * p0.x - 5.0f * p1.x + 4.0f * p2.x - p3.x) * t2 +
//                                 (-p0.x + 3.0f * p1.x - 3.0f * p2.x + p3.x) * t3);
        
//         interpolated.y = 0.5f * ((2.0f * p1.y) +
//                                 (-p0.y + p2.y) * t_local +
//                                 (2.0f * p0.y - 5.0f * p1.y + 4.0f * p2.y - p3.y) * t2 +
//                                 (-p0.y + 3.0f * p1.y - 3.0f * p2.y + p3.y) * t3);
        
//         interpolated.z = 0.5f * ((2.0f * p1.z) +
//                                 (-p0.z + p2.z) * t_local +
//                                 (2.0f * p0.z - 5.0f * p1.z + 4.0f * p2.z - p3.z) * t2 +
//                                 (-p0.z + 3.0f * p1.z - 3.0f * p2.z + p3.z) * t3);
        
//         interpolated.yaw = 0.0f; // Will be calculated later with auto-heading
        
//         smooth_path.push_back(interpolated);
//     }
    
//     return smooth_path;
// }

// std::vector<PathWaypoint> ramerDouglasPeucker(const std::vector<PathWaypoint>& path, float epsilon) {
//     /**
//      * Ramer-Douglas-Peucker path simplification
//      * Direct port from Python implementation
//      */
    
//     if (path.size() < 3) {
//         return path;
//     }
    
//     // Find point with maximum perpendicular distance
//     PathWaypoint start = path.front();
//     PathWaypoint end = path.back();
    
//     float dx = end.x - start.x;
//     float dy = end.y - start.y;
//     float dz = end.z - start.z;
//     float line_len = sqrt(dx*dx + dy*dy + dz*dz);
    
//     float dmax = 0.0f;
//     size_t index = 0;
    
//     if (line_len > 1e-10f) {
//         float dir_x = dx / line_len;
//         float dir_y = dy / line_len;
//         float dir_z = dz / line_len;
        
//         for (size_t i = 1; i < path.size() - 1; i++) {
//             float px = path[i].x - start.x;
//             float py = path[i].y - start.y;
//             float pz = path[i].z - start.z;
            
//             // Project onto line
//             float t = std::clamp(px*dir_x + py*dir_y + pz*dir_z, 0.0f, line_len);
            
//             // Closest point on line
//             float closest_x = start.x + t * dir_x;
//             float closest_y = start.y + t * dir_y;
//             float closest_z = start.z + t * dir_z;
            
//             // Perpendicular distance
//             float dist_x = path[i].x - closest_x;
//             float dist_y = path[i].y - closest_y;
//             float dist_z = path[i].z - closest_z;
//             float d = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
            
//             if (d > dmax) {
//                 dmax = d;
//                 index = i;
//             }
//         }
//     } else { 
//         for (size_t i = 1; i < path.size() - 1; i++) {
//             float dx = path[i].x - start.x;
//             float dy = path[i].y - start.y;
//             float dz = path[i].z - start.z;
//             float d = sqrt(dx*dx + dy*dy + dz*dz);
//             if (d > dmax) {
//                 dmax = d;
//                 index = i;
//             }
//         }
//     }
    
//     // Recursive simplification
//     std::vector<PathWaypoint> result;
//     if (dmax > epsilon) {
//         std::vector<PathWaypoint> left(path.begin(), path.begin() + index + 1);
//         std::vector<PathWaypoint> right(path.begin() + index, path.end());
        
//         auto left_result = ramerDouglasPeucker(left, epsilon);
//         auto right_result = ramerDouglasPeucker(right, epsilon);
        
//         result.insert(result.end(), left_result.begin(), left_result.end() - 1);
//         result.insert(result.end(), right_result.begin(), right_result.end());
//     } else {
//         result.push_back(start);
//         result.push_back(end);
//     }
    
//     return result;
// }

// void followPathVelocity(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, const std::vector<PathWaypoint>& path, float max_speed, float step_tolerance, float slow_down_distance, bool enable_lateral_comp, float lateral_comp_factor) {
    
//     RCLCPP_INFO(node->get_logger(), "Following %zu waypoints | Speed: %.1f m/s | Tol: %.2fm", 
//                 path.size(), max_speed, step_tolerance);
//     RCLCPP_INFO(node->get_logger(), "Horizontal navigation only - altitude locked (vel_z = 0)");
    
//     geometry_msgs::msg::Twist vel_cmd;
//     const float MAX_TIME_PER_WP = 60.0f; // 60 seconds timeout per waypoint
    
//     for (size_t i = 0; i < path.size(); i++) {
//         const PathWaypoint& target = path[i];
        
//         auto start_time = std::chrono::steady_clock::now();
//         int loop_count = 0;
        
//         while (rclcpp::ok()) {
//             auto curr_pose = node->getCurrentLocalPose();
            
//             // Calculate HORIZONTAL direction vector only (ignore target.z)
//             float dx = target.x - curr_pose.pose.position.x;
//             float dy = target.y - curr_pose.pose.position.y;
//             float horizontal_error = sqrt(dx*dx + dy*dy);
            
//             // Check timeout
//             auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
//                 std::chrono::steady_clock::now() - start_time).count();
//             if (elapsed > MAX_TIME_PER_WP) {
//                 RCLCPP_WARN(node->get_logger(), "WP %zu timeout (%.1fs) - skipping", i+1, (float)elapsed);
//                 break;
//             }
            
//             // Check if waypoint reached (HORIZONTAL distance only)
//             if (horizontal_error < step_tolerance) {
//                 if (loop_count % 10 == 0 || loop_count < 5) {
//                     RCLCPP_INFO(node->get_logger(), "WP %zu/%zu reached (horizontal err: %.2fm)", 
//                                 i+1, path.size(), horizontal_error);
//                 }
//                 break;
//             }
            
//             // Set HORIZONTAL velocity (X, Y) only
//             vel_cmd.linear.x = -dx;
//             vel_cmd.linear.y = -dy;
//             vel_cmd.linear.z = 0.0f; 
            
//             // Lateral compensation
//             if (enable_lateral_comp && fabs(vel_cmd.linear.y) >= 1.0f) {
//                 vel_cmd.linear.x = -dx / (lateral_comp_factor * fabs(vel_cmd.linear.y));
//             }
            
//             // Clamp HORIZONTAL velocity to max speed
//             if (horizontal_error > max_speed) {
//                 float scale = max_speed / horizontal_error;
//                 vel_cmd.linear.x *= scale;
//                 vel_cmd.linear.y *= scale;
//                 // vel_z stays 0
//             }
            
//             // Auto slow-down near endpoint (HORIZONTAL only)
//             float dist_to_end_x = path.back().x - target.x;
//             float dist_to_end_y = path.back().y - target.y;
//             float horizontal_dist_to_end = sqrt(dist_to_end_x*dist_to_end_x + 
//                                                 dist_to_end_y*dist_to_end_y);
            
//             if (horizontal_dist_to_end < slow_down_distance && i > path.size() / 2) {
//                 float speed_factor = std::max(0.1f, horizontal_dist_to_end / slow_down_distance);
//                 vel_cmd.linear.x *= speed_factor;
//                 vel_cmd.linear.y *= speed_factor;
//                 // vel_z stays 0
//             }
            
//             // Publish velocity command
//             node->publishLocalVelocityUnstamped(vel_cmd);
            
//             // Log progress every 20 loops (reduce spam)
//             if (loop_count % 20 == 0) {
//                 RCLCPP_INFO(node->get_logger(), "WP %zu: h_err=%.2fm vel_xy=[%.2f,%.2f] alt=%.2fm", 
//                            i+1, horizontal_error, vel_cmd.linear.x, vel_cmd.linear.y,
//                            curr_pose.pose.position.z);
//             }
            
//             loop_count++;
//             rclcpp::spin_some(node);
//             rate.sleep();
//         }
//     }

//     vel_cmd.linear.x = 0.0f;
//     vel_cmd.linear.y = 0.0f;
//     vel_cmd.linear.z = 0.0f;
//     node->publishLocalVelocityUnstamped(vel_cmd); 
    
//     posee = node->getCurrentLocalPose();
//     RCLCPP_INFO(node->get_logger(), "Path complete! Final: [%.2f, %.2f, %.2f]", 
//                 posee.pose.position.x, posee.pose.position.y, posee.pose.position.z);
// }

// void followPathVelocity(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, const std::vector<PathWaypoint>& path, float max_speed, float step_tolerance, float slow_down_distance, bool enable_lateral_comp, float lateral_comp_factor) {
    
//     RCLCPP_INFO(node->get_logger(), "Following %zu waypoints | Speed: %.1f m/s | Tol: %.2fm", 
//                 path.size(), max_speed, step_tolerance);
//     RCLCPP_INFO(node->get_logger(), "Horizontal navigation only - altitude locked (vel_z = 0)");
//     RCLCPP_INFO(node->get_logger(), "Using GLOBAL frame velocity (map) - heading remains constant");
    
//     geometry_msgs::msg::TwistStamped vel_cmd;
//     const float MAX_TIME_PER_WP = 60.0f; // 60 seconds timeout per waypoint
    
//     for (size_t i = 0; i < path.size(); i++) {
//         const PathWaypoint& target = path[i];
        
//         auto start_time = std::chrono::steady_clock::now();
//         int loop_count = 0;
        
//         while (rclcpp::ok()) {
//             auto curr_pose = node->getCurrentLocalPose();
            
//             // Calculate HORIZONTAL direction vector only (ignore target.z)
//             float dx = target.x - curr_pose.pose.position.x;
//             float dy = target.y - curr_pose.pose.position.y;
//             float horizontal_error = sqrt(dx*dx + dy*dy);
            
//             // Check timeout
//             auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
//                 std::chrono::steady_clock::now() - start_time).count();
//             if (elapsed > MAX_TIME_PER_WP) {
//                 RCLCPP_WARN(node->get_logger(), "WP %zu timeout (%.1fs) - skipping", i+1, (float)elapsed);
//                 break;
//             }

//             if (horizontal_error < step_tolerance) {
//                 if (loop_count % 10 == 0 || loop_count < 5) {
//                     RCLCPP_INFO(node->get_logger(), "WP %zu/%zu reached (horizontal err: %.2fm)", 
//                                 i+1, path.size(), horizontal_error);
//                 }
//                 break;
//             }
            
//             vel_cmd.header.stamp = node->now();
//             vel_cmd.header.frame_id = "map";  
//             vel_cmd.twist.linear.x = dx;
//             vel_cmd.twist.linear.y = dy;
//             vel_cmd.twist.linear.z = 0.0f; 

//             if (enable_lateral_comp && fabs(vel_cmd.twist.linear.y) >= 1.0f) {
//                 vel_cmd.twist.linear.x = dx / (lateral_comp_factor * fabs(vel_cmd.twist.linear.y));
//             }
            
//             if (horizontal_error > max_speed) {
//                 float scale = max_speed / horizontal_error;
//                 vel_cmd.twist.linear.x *= scale;
//                 vel_cmd.twist.linear.y *= scale;
//             }

//             float dist_to_end_x = path.back().x - target.x;
//             float dist_to_end_y = path.back().y - target.y;
//             float horizontal_dist_to_end = sqrt(dist_to_end_x*dist_to_end_x + 
//                                                 dist_to_end_y*dist_to_end_y);
            
//             if (horizontal_dist_to_end < slow_down_distance && i > path.size() / 2) {
//                 float speed_factor = std::max(0.1f, horizontal_dist_to_end / slow_down_distance);
//                 vel_cmd.twist.linear.x *= speed_factor;
//                 vel_cmd.twist.linear.y *= speed_factor;
//                 // vel_z stays 0
//             }
            
//             // Publish velocity command in GLOBAL frame
//             node->publishLocalVelocity(vel_cmd);
            
//             // Log progress every 20 loops (reduce spam)
//             if (loop_count % 20 == 0) {
//                 RCLCPP_INFO(node->get_logger(), "WP %zu: h_err=%.2fm vel_map=[%.2f,%.2f] alt=%.2fm", 
//                            i+1, horizontal_error, vel_cmd.twist.linear.x, vel_cmd.twist.linear.y,
//                            curr_pose.pose.position.z);
//             }
            
//             loop_count++;
//             rclcpp::spin_some(node);
//             rate.sleep();
//         }
//     }

//     // Stop velocity
//     vel_cmd.header.stamp = node->now();
//     vel_cmd.header.frame_id = "map";
//     vel_cmd.twist.linear.x = 0.0f;
//     vel_cmd.twist.linear.y = 0.0f;
//     vel_cmd.twist.linear.z = 0.0f;
//     node->publishLocalVelocity(vel_cmd); 
    
//     posee = node->getCurrentLocalPose();
//     RCLCPP_INFO(node->get_logger(), "Path complete! Final: [%.2f, %.2f, %.2f]", 
//                 posee.pose.position.x, posee.pose.position.y, posee.pose.position.z);
// }

// void followPathFromConfig(const std::shared_ptr<DroneController>&node, rclcpp::Rate &rate, geometry_msgs::msg::PoseStamped &posee, const PathConfig& config) {
    
//     RCLCPP_INFO(node->get_logger(), "Path: %zu waypoints | Speed: %.1f m/s | Tol: %.2fm", 
//                 config.waypoints.size(), config.speed, config.tolerance);
    
//     std::vector<PathWaypoint> path_to_follow = config.waypoints;
//     // 
//     if (config.enable_bspline_smoothing && config.waypoints.size() >= 2) {
//         path_to_follow = smoothPathBSpline(config.waypoints, config.bspline_smoothness,
//                                             config.bspline_degree, config.num_interpolation_points);
//         RCLCPP_INFO(node->get_logger(), "B-Spline: %zu → %zu points", 
//                     config.waypoints.size(), path_to_follow.size());
//     }
    
//     // RDP simplification
//     if (config.enable_rdp_simplification && path_to_follow.size() > 10) {
//         auto simplified = ramerDouglasPeucker(path_to_follow, config.rdp_epsilon);
//         RCLCPP_INFO(node->get_logger(), "RDP: %zu --> %zu points", 
//                     path_to_follow.size(), simplified.size());
//         path_to_follow = simplified;
//     }
    
//     // Auto-heading
//     if (config.auto_heading && path_to_follow.size() > 1) {
//         for (size_t i = 0; i < path_to_follow.size() - 1; i++) {
//             float dx = path_to_follow[i+1].x - path_to_follow[i].x;
//             float dy = path_to_follow[i+1].y - path_to_follow[i].y;
//             path_to_follow[i].yaw = atan2(dy, dx);
//         }
//         path_to_follow.back().yaw = path_to_follow[path_to_follow.size()-2].yaw;
//     }
    
//     // Follow path
//     followPathVelocity(node, rate, posee, path_to_follow, config.speed, config.tolerance,
//                       config.slow_down_distance, config.enable_lateral_compensation,
//                       config.lateral_compensation_factor);
// }
