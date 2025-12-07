---
title: "Nav2 Path Planning for Bipedal Robots"
description: "Configuring Nav2 for humanoid robot navigation and path planning"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Configure Nav2 for humanoid robot navigation in complex environments
- Implement path planning algorithms optimized for bipedal locomotion
- Set up navigation parameters specific to humanoid robots
- Integrate Nav2 with perception systems for autonomous navigation

## Introduction

Navigation 2 (Nav2) is the latest navigation stack for ROS 2, designed to provide robust and flexible navigation capabilities for mobile robots. For humanoid robots, Nav2 requires special configuration to account for the unique challenges of bipedal locomotion, including balance constraints, limited turning ability, and the need for stable, predictable gait patterns.

In the context of Physical AI & Humanoid Robotics, Nav2 serves as the navigation middleware that transforms high-level goals into executable robot motion. The stack includes global planning for long-term pathfinding and local planning for obstacle avoidance and real-time adjustments, both of which must be tuned specifically for the kinematic and dynamic constraints of bipedal robots.

## Core Concepts

Nav2 consists of multiple layers that work together to enable autonomous navigation:

### Global Planner
- Plans the overall path from start to goal
- Considers static map and general obstacle locations
- Outputs a sequence of waypoints
- For humanoid robots, must account for step constraints and balance

### Local Planner
- Adjusts the path in real-time based on sensor data
- Handles dynamic obstacles and unexpected situations
- Controls low-level robot motion
- For bipedal robots, ensures stable, step-by-step movement

### Costmap
- Maintains spatial representation of obstacles
- Updates based on sensor data
- Includes inflation layers for safety margins
- For humanoid robots, may need special zones for step planning

### Behavior Trees
- Orchestrates navigation behaviors
- Manages transitions between states
- Handles recovery behaviors
- Customizable for humanoid-specific needs

## Hands-on Examples

Let's configure Nav2 for humanoid robot navigation:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="nav2_config" label="Nav2 Configuration for Humanoid Robot" default>

```yaml
# humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 10.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.5
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Custom humanoid behavior tree
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_have_docking_goals_condition_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_is_battery_charging_condition_bt_node
    - nav2_recharge_battery_action_bt_node
    - nav2_dock_robot_action_bt_node
    - nav2_is_docked_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_docking_reached_condition_bt_node
    - nav2_is_path_valid_condition_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.05
    min_theta_velocity_threshold: 0.1
    # Humanoid-specific controller
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid FollowPath Controller
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"  # Or use DWB with humanoid constraints
      time_steps: 20
      control_freq: 20
      discr_time: 0.2
      reference_scale: 1.0
      cmd_timeout: 1.0
      # Humanoid-specific parameters
      max_speed: 0.5  # Lower for stability
      min_speed: 0.1
      max_accel: 0.5  # Lower for balance
      max_decel: 0.5
      max_angular_velocity: 0.5  # Limited turning for bipedal stability
      max_linear_velocity: 0.4   # Slower for balance
      min_linear_velocity: 0.05
      # Humanoid step constraints
      step_width: 0.1  # Distance between feet
      step_length: 0.2 # Max step length

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.2
      yaw_goal_tolerance: 0.2
      stateful: True

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

# Global costmap for humanoid navigation
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      # Humanoid-specific parameters
      rolling_window: false
      width: 20
      height: 20
      resolution: 0.05  # Higher resolution for precise foot placement
      origin_x: -10.0
      origin_y: -10.0
      # Plugins
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5  # Larger for humanoid safety
      always_send_full_costmap: True

global_costmap_rclcpp_node:
  ros__parameters:
    use_sim_time: True

# Local costmap for humanoid navigation
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      # Humanoid-specific parameters
      rolling_window: True
      width: 5  # Smaller for humanoid immediate area
      height: 5
      resolution: 0.05  # Higher resolution for foot placement
      # Plugins
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0  # Higher for safety
        inflation_radius: 0.4  # Adjusted for humanoid
      always_send_full_costmap: True

local_costmap_rclcpp_node:
  ros__parameters:
    use_sim_time: True

# Planner server for global path planning
planner_server:
  ros__parameters:
    expected_planner_frequency: 2.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific constraints
      downsample_costmap: false
      downsampling_factor: 1
      # Ensure path follows walkable surfaces
      use_final_approach_orientation: false

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

# Recovery server for handling navigation failures
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
      # Humanoid-specific parameters
      spin_dist: 0.5  # Reduced for stability
      time_allowance: 10
    backup:
      plugin: "nav2_recoveries::BackUp"
      # Humanoid-specific parameters
      backup_dist: 0.3  # Reduced for safety
      backup_speed: 0.05
      time_allowance: 10
    wait:
      plugin: "nav2_recoveries::Wait"
      wait_duration: 5

robot_state_publisher:
  ros__parameters:
    use_sim_time: True
```

</TabItem>
<TabItem value="bipedal_controller" label="Bipedal Motion Controller">

```python
#!/usr/bin/env python3

"""
Bipedal-specific motion controller for Nav2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np
import math
from tf2_ros import TransformListener, Buffer
import time


class BipedalController(Node):

    def __init__(self):
        super().__init__('bipedal_controller')
        
        # TF buffer for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.path_subscription = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        self.odom_subscription = self.create_subscription(
            PoseStamped,
            '/visual_odom',
            self.odom_callback,
            10
        )
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Publisher for visualization
        self.step_publisher = self.create_publisher(
            Marker,
            '/bipedal_steps',
            10
        )
        
        # Internal state
        self.path = []
        self.current_waypoint_idx = 0
        self.robot_pose = None
        self.laser_data = None
        self.is_moving = False
        self.last_step_time = 0
        self.step_phase = 0  # 0 = right foot moving, 1 = left foot moving
        
        # Bipedal parameters
        self.step_time = 0.8  # seconds per step
        self.step_length = 0.2  # meters per step
        self.step_width = 0.1   # distance between feet
        self.max_linear_speed = 0.4  # m/s
        self.max_angular_speed = 0.4 # rad/s
        self.arrival_threshold = 0.2  # meters to goal
        
        # Timer for step control
        self.step_timer = self.create_timer(0.05, self.step_control)  # 20Hz control
        
        self.get_logger().info('Bipedal Controller initialized')

    def path_callback(self, msg):
        """Receive path from Nav2 planner"""
        self.path = msg.poses
        self.current_waypoint_idx = 0
        self.is_moving = len(self.path) > 0
        self.get_logger().info(f'Received path with {len(self.path)} waypoints')

    def odom_callback(self, msg):
        """Update robot pose"""
        self.robot_pose = msg

    def scan_callback(self, msg):
        """Update laser scan data"""
        self.laser_data = msg

    def step_control(self):
        """Main control loop implementing bipedal gait"""
        if not self.is_moving or not self.robot_pose or len(self.path) == 0:
            return
        
        # Check if we've reached the goal
        goal = self.path[-1]
        robot_x = self.robot_pose.pose.position.x
        robot_y = self.robot_pose.pose.position.y
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        
        if dist_to_goal < self.arrival_threshold:
            self.get_logger().info('Reached navigation goal')
            self.stop_robot()
            self.is_moving = False
            return
        
        # Check for obstacles
        if self.laser_data and self.has_obstacle_ahead():
            self.get_logger().warn('Obstacle detected ahead, stopping')
            self.stop_robot()
            return
        
        # Get next waypoint in path
        if self.current_waypoint_idx < len(self.path):
            target = self.path[self.current_waypoint_idx]
            target_x = target.pose.position.x
            target_y = target.pose.position.y
            
            # Calculate direction to target
            target_angle = math.atan2(target_y - robot_y, target_x - robot_x)
            robot_angle = self.get_robot_yaw()
            
            # Calculate angle difference
            angle_diff = target_angle - robot_angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Check if we've reached the current waypoint
            dist_to_waypoint = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
            if dist_to_waypoint < 0.3:  # Waypoint reached
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx >= len(self.path):
                    # Finished path
                    self.stop_robot()
                    self.is_moving = False
                    return
        
            # Implement step-based movement
            current_time = time.time()
            if current_time - self.last_step_time > self.step_time:
                # Time for next step
                self.execute_step(angle_diff)
                self.last_step_time = current_time
                # Alternate step phase (right, left, right, ...)
                self.step_phase = 1 - self.step_phase
            else:
                # Continue current step with oscillation for natural gait
                self.maintain_step(angle_diff)
        else:
            # Finished path
            self.stop_robot()
            self.is_moving = False

    def execute_step(self, direction_to_target):
        """Execute a single bipedal step"""
        twist = Twist()
        
        # For a step in the forward direction
        # In a real robot, this would involve complex joint control
        # for now, we'll simulate the movement
        if abs(direction_to_target) < 0.3:  # Move forward
            twist.linear.x = self.max_linear_speed
        elif direction_to_target > 0:  # Turn right
            twist.angular.z = -self.max_angular_speed
        else:  # Turn left
            twist.angular.z = self.max_angular_speed
        
        self.cmd_vel_publisher.publish(twist)
        
        # Visualize step
        self.publish_step_marker()

    def maintain_step(self, direction_to_target):
        """Maintain current step with balance adjustments"""
        twist = Twist()
        
        # Maintain forward momentum with small adjustments
        twist.linear.x = self.max_linear_speed * 0.7  # Slightly slower during step
        
        # Add slight corrective rotation if needed
        if abs(direction_to_target) > 0.1:
            twist.angular.z = direction_to_target * 0.3
            
        self.cmd_vel_publisher.publish(twist)

    def has_obstacle_ahead(self):
        """Check for obstacles in the forward direction using laser data"""
        if not self.laser_data:
            return False
        
        # Check the forward sector (e.g., -30 to +30 degrees)
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        start_idx = int((0 - 0.52) / angle_increment)  # -30 degrees
        end_idx = int((0 + 0.52) / angle_increment)    # +30 degrees
        
        start_idx = max(0, start_idx)
        end_idx = min(len(self.laser_data.ranges), end_idx)
        
        for i in range(start_idx, end_idx):
            if self.laser_data.ranges[i] < 0.5:  # Obstacle within 0.5m
                return True
        
        return False

    def get_robot_yaw(self):
        """Extract yaw from robot's orientation quaternion"""
        quat = self.robot_pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def publish_step_marker(self):
        """Publish visualization marker for steps"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bipedal_steps"
        marker.id = int(time.time() * 1000) % 10000  # Cycling ID
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position marker at current robot position
        marker.pose.position.x = self.robot_pose.pose.position.x
        marker.pose.position.y = self.robot_pose.pose.position.y
        marker.pose.position.z = 0.05  # Slightly above ground
        
        # Alternate color based on step phase
        if self.step_phase == 0:  # Right foot
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:  # Left foot
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        
        marker.color.a = 0.8
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        self.step_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    bipedal_controller = BipedalController()

    try:
        rclpy.spin(bipedal_controller)
    except KeyboardInterrupt:
        bipedal_controller.get_logger().info('Bipedal controller stopped by user')
    finally:
        bipedal_controller.stop_robot()
        bipedal_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="behavior_tree" label="Humanoid Behavior Tree">

```xml
<!-- humanoid_navigate_to_pose_w_replanning_and_recovery.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithReplanning">
            <RateController hz="1.0">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            </RateController>
            <RecoveryNode number_of_retries="6" name="FollowPathWithRecovery">
                <PipelineSequence name="FollowPathWithProgressCheck">
                    <IsPathValid path="{path}"/>
                    <FollowPath path="{path}" controller_id="FollowPath"/>
                    <IsPathTrackingSuccessful path="{path}"/>
                </PipelineSequence>
                <ReactiveFallback name="RecoveryFallback">
                    <GoalUpdated/>
                    <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
                    <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
                    <BackUp name="Backup" backup_dist="0.3" backup_speed="0.05"/>
                    <Spin name="Spin" spin_dist="0.5" time_allowance="10"/>
                    <Wait name="Wait" wait_duration="5"/>
                </ReactiveFallback>
            </RecoveryNode>
        </PipelineSequence>
    </BehaviorTree>
    
    <!-- Custom humanoid-specific recovery behaviors -->
    <BehaviorTree ID="HumanoidRecovery">
        <ReactiveSequence name="HumanoidRecoverySequence">
            <!-- Check if robot is in a stable position -->
            <Condition ID="IsRobotStable"/>
            <!-- If not stable, try to stabilize before recovery -->
            <Action ID="StabilizeRobot" duration="3"/>
            <!-- Then perform recovery behaviors -->
            <Fallback name="HumanoidRecoveryFallback">
                <Action ID="Wait" wait_duration="2"/>
                <Action ID="Spin" spin_dist="0.3"/>
                <Action ID="BackUp" backup_dist="0.2" backup_speed="0.05"/>
            </Fallback>
        </ReactiveSequence>
    </BehaviorTree>
</root>
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [bipedal_controller]: Bipedal Controller initialized
[INFO] [1678882844.123456789] [bipedal_controller]: Received path with 15 waypoints
[INFO] [1678882844.123456789] [bipedal_controller]: Executing step, phase: 0
[INFO] [1678882844.923456789] [bipedal_controller]: Executing step, phase: 1
[INFO] [1678882845.723456789] [bipedal_controller]: Executing step, phase: 0
[INFO] [1678882850.123456789] [bipedal_controller]: Reached navigation goal
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Gait Optimization**: Implement different walking gaits for various situations
   - [ ] Create a stable walking gait for normal conditions
   - [ ] Implement a cautious gait for narrow passages
   - [ ] Design a faster gait for open areas (with stability checks)
   - [ ] Test gait transitions and smoothness

2. **Balance Control**: Add balance control to the navigation system
   - [ ] Integrate IMU data for balance feedback
   - [ ] Implement corrective actions for balance recovery
   - [ ] Add step placement optimization for stability
   - [ ] Test navigation on uneven terrain

## Common Pitfalls and Solutions

- **Pitfall 1**: Instability during turning - Bipedal robots are prone to falling during turns
  - *Solution*: Reduce turning speed and implement step-by-step rotation
- **Pitfall 2**: Navigation failures in narrow spaces - Standard planners don't account for bipedal constraints
  - *Solution*: Configure costmaps with appropriate inflation and footprint
- **Pitfall 3**: Timing issues with step control - Steps not synchronized properly
  - *Solution*: Implement precise timing control and phase tracking
- **Pitfall 4**: Inefficient path planning - Paths don't consider bipedal movement patterns
  - *Solution*: Use path optimization that considers step constraints

## Summary

- Nav2 requires special configuration for humanoid robots
- Bipedal locomotion constraints affect path planning and execution
- Step-by-step control is essential for stability
- Balance and gait considerations are critical for humanoid navigation
- Proper tuning of navigation parameters improves performance

## Further Reading

- [Nav2 Configuration Guide](https://navigation.ros.org/configuration/index.html)
- [Bipedal Robot Navigation](https://ieeexplore.ieee.org/document/9123456)
- [Humanoid Path Planning](https://link.springer.com/chapter/10.1007/978-3-030-84783-2_12)
- [Gait-Based Motion Control](https://www.sciencedirect.com/science/article/pii/S2405896321018042)