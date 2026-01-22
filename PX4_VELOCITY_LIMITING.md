# PX4 Velocity Limiting - Solusi untuk Jarak Jauh

## Problem
Ketika menggunakan `executeLocalWaypointMove()` dengan jarak jauh (misal 10 meter), drone PX4 **tidak terbang sejauh yang diharapkan** seperti di ArduPilot. Drone hanya **bergerak lebih cepat** tapi jarak tidak sesuai.

## Penyebab
PX4 memiliki **velocity limiting** internal di OFFBOARD mode untuk keamanan:
- `MPC_XY_VEL_MAX` - Max horizontal velocity (default: 12 m/s)
- `MPC_Z_VEL_MAX_UP` - Max vertical velocity up (default: 3 m/s)  
- `MPC_XY_CRUISE` - Normal cruise velocity (default: 5 m/s)
- `MPC_JERK_MAX` - Max jerk limit (default: 8 m/s³)
- `MPC_ACC_HOR_MAX` - Max horizontal acceleration (default: 5 m/s²)

**MASALAH UTAMA**: PX4 menggunakan **trajectory smoothing** yang membatasi akselerasi dan jerk, sehingga untuk jarak jauh, drone tidak langsung terbang full speed tapi secara gradual.

## Solusi 1: Tingkatkan PX4 Velocity Parameters (RECOMMENDED)

Jalankan command berikut di QGroundControl atau MAVLink shell:

```bash
# Tingkatkan max velocity horizontal
param set MPC_XY_VEL_MAX 15.0      # Default: 12 m/s
param set MPC_XY_CRUISE 8.0        # Default: 5 m/s

# Tingkatkan akselerasi untuk respon lebih cepat
param set MPC_ACC_HOR_MAX 8.0      # Default: 5 m/s²
param set MPC_ACC_HOR 8.0          # Default: 3 m/s²

# Tingkatkan jerk untuk transisi lebih agresif
param set MPC_JERK_MAX 12.0        # Default: 8 m/s³
param set MPC_JERK_AUTO 12.0       # Default: 4 m/s³

# Save parameters
param save
```

Atau via MAVROS service (tambahkan di code):

```cpp
// Set PX4 velocity parameters for faster long-distance movement
setParam(node, "MPC_XY_VEL_MAX", 15.0);
setParam(node, "MPC_XY_CRUISE", 8.0);
setParam(node, "MPC_ACC_HOR_MAX", 8.0);
setParam(node, "MPC_JERK_MAX", 12.0);
```

## Solusi 2: Gunakan Velocity Control dengan Target yang Update

Ubah `executeLocalWaypointMove()` untuk menggunakan **velocity setpoints** langsung:

```cpp
void executeLocalWaypointMoveVelocity(const std::shared_ptr<DroneController>&node, 
                                      float forward_x, float left_y, float up_z, 
                                      float max_velocity = 5.0, float tolerance = 0.3) {
    // Calculate target position
    geometry_msgs::msg::PoseStamped current_pose = node->getCurrentLocalPose();
    float current_yaw = getHeading(current_pose.pose.orientation);
    
    float global_x = forward_x * cos(current_yaw) - left_y * sin(current_yaw);
    float global_y = forward_x * sin(current_yaw) + left_y * cos(current_yaw);
    
    float target_x = current_pose.pose.position.x + global_x;
    float target_y = current_pose.pose.position.y + global_y;
    float target_z = current_pose.pose.position.z + up_z;
    
    rclcpp::Rate fast_rate(20.0);
    
    while (rclcpp::ok()) {
        current_pose = node->getCurrentLocalPose();
        
        // Calculate error
        float error_x = target_x - current_pose.pose.position.x;
        float error_y = target_y - current_pose.pose.position.y;
        float error_z = target_z - current_pose.pose.position.z;
        
        float distance = sqrt(error_x*error_x + error_y*error_y + error_z*error_z);
        
        if (distance < tolerance) {
            RCLCPP_INFO(node->get_logger(), "Target reached!");
            break;
        }
        
        // Calculate velocity (proportional to error, limited by max_velocity)
        float vel_x = error_x / distance * max_velocity;
        float vel_y = error_y / distance * max_velocity;
        float vel_z = error_z / distance * max_velocity;
        
        // Slow down when approaching target
        if (distance < 1.0) {
            float scale = distance / 1.0;
            vel_x *= scale;
            vel_y *= scale;
            vel_z *= scale;
        }
        
        // Publish velocity command
        geometry_msgs::msg::TwistStamped vel_cmd;
        vel_cmd.header.stamp = node->now();
        vel_cmd.header.frame_id = "map";
        vel_cmd.twist.linear.x = vel_x;
        vel_cmd.twist.linear.y = vel_y;
        vel_cmd.twist.linear.z = vel_z;
        
        node->publishLocalVelocity(vel_cmd);
        
        rclcpp::spin_some(node);
        fast_rate.sleep();
    }
    
    // Stop
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.stamp = node->now();
    stop_cmd.header.frame_id = "map";
    node->publishLocalVelocity(stop_cmd);
}
```

## Solusi 3: Gunakan PositionTarget dengan Velocity Mask

Gunakan `/mavros/setpoint_raw/local` dengan velocity feed-forward:

```cpp
void executeLocalWaypointMoveRaw(const std::shared_ptr<DroneController>&node,
                                 float forward_x, float left_y, float up_z,
                                 float cruise_velocity = 5.0) {
    // Similar calculation...
    
    mavros_msgs::msg::PositionTarget target;
    target.header.stamp = node->now();
    target.header.frame_id = "map";
    target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    
    // Enable position + velocity control
    target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE |
                       mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                       mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                       mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
    
    target.position.x = target_x;
    target.position.y = target_y;
    target.position.z = target_z;
    
    // Add velocity feed-forward for faster response
    target.velocity.x = (target_x - current_x) / abs(target_x - current_x) * cruise_velocity;
    target.velocity.y = (target_y - current_y) / abs(target_y - current_y) * cruise_velocity;
    target.velocity.z = (target_z - current_z) / abs(target_z - current_z) * cruise_velocity;
    
    node->publishSetpointRaw(target); // Publish at 20Hz
}
```

## Solusi 4: Breaking Long Distance into Waypoints

Untuk jarak sangat jauh (>10m), bagi menjadi waypoint intermediate:

```cpp
void executeLocalWaypointMoveLongDistance(const std::shared_ptr<DroneController>&node,
                                          float forward_x, float left_y, float up_z,
                                          float segment_length = 5.0, float tolerance = 0.3) {
    float total_distance = sqrt(forward_x*forward_x + left_y*left_y + up_z*up_z);
    int num_segments = (int)ceil(total_distance / segment_length);
    
    float segment_forward = forward_x / num_segments;
    float segment_left = left_y / num_segments;
    float segment_up = up_z / num_segments;
    
    RCLCPP_INFO(node->get_logger(), "Breaking %.2fm into %d segments of %.2fm each",
                total_distance, num_segments, segment_length);
    
    for (int i = 0; i < num_segments; i++) {
        RCLCPP_INFO(node->get_logger(), "Segment %d/%d", i+1, num_segments);
        executeLocalWaypointMove(node, rate, posee, 
                                segment_forward, segment_left, segment_up, 
                                0.0, tolerance);
    }
}
```

## Rekomendasi

**Untuk jarak pendek (<5m)**: Gunakan fungsi existing `executeLocalWaypointMove()` sudah cukup

**Untuk jarak menengah (5-10m)**: Tingkatkan PX4 parameters (Solusi 1)

**Untuk jarak jauh (>10m)**: Kombinasi Solusi 1 + Solusi 4 (breaking waypoints)

**Untuk kontrol paling presisi**: Gunakan Solusi 2 (velocity control) dengan tuning yang baik

## Testing

Test dengan berbagai jarak:
```bash
# Edit config.yaml
forward_distance: 3.0   # Should work fine
forward_distance: 10.0  # Apply Solution 1 or 4
forward_distance: 20.0  # Must use Solution 4
```

Monitor dengan:
```bash
ros2 topic echo /mavros/local_position/pose
ros2 topic echo /mavros/setpoint_position/local
```
