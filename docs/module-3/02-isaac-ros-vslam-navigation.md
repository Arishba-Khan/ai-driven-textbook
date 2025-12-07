---
title: "Isaac ROS: Visual SLAM and Navigation"
description: "Implementing Visual SLAM and navigation with Isaac ROS packages"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Set up and configure Isaac ROS packages for Visual SLAM
- Implement Visual SLAM pipelines using Isaac Sim and ROS 2
- Configure navigation systems in Isaac Sim for humanoid robots
- Integrate perception and navigation for autonomous operation

## Introduction

Isaac ROS represents NVIDIA's collection of hardware-accelerated ROS 2 packages specifically designed for robotics applications. These packages leverage NVIDIA's GPU computing capabilities to accelerate perception, navigation, and control tasks. For Physical AI & Humanoid Robotics applications, Isaac ROS packages provide significant performance improvements for computationally intensive tasks like Visual SLAM (Simultaneous Localization and Mapping), which are critical for autonomous robot operation.

Visual SLAM combines visual perception with localization and mapping to allow robots to understand their environment and navigate autonomously. In the context of Isaac Sim, Visual SLAM systems can be developed and tested in photorealistic environments before deployment on real hardware, significantly accelerating the development process.

## Core Concepts

Isaac ROS packages are built to interoperate with the broader ROS 2 ecosystem while leveraging NVIDIA's GPU acceleration for performance. The packages include:

### Isaac ROS Common
- Hardware acceleration wrappers
- Message type definitions
- Performance optimization utilities
- Integration with NVIDIA tools

### Isaac ROS Visual SLAM
- Hardware-accelerated visual-inertial odometry (VIO)
- Feature tracking and matching
- Map building and optimization
- Loop closure detection

### Isaac ROS Navigation
- GPU-accelerated path planning
- Costmap management
- Local and global planners
- Obstacle avoidance algorithms

### GPU Acceleration
Isaac ROS packages leverage CUDA and TensorRT to accelerate:
- Deep learning inference
- Image processing operations
- Feature detection and matching
- Path planning algorithms

## Hands-on Examples

Let's implement Visual SLAM and navigation with Isaac ROS:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="isaac_slam" label="Isaac ROS Visual SLAM Setup" default>

```python
#!/usr/bin/env python3

"""
Isaac ROS Visual SLAM Node for Humanoid Robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point


class IsaacVisualSLAMNode(Node):

    def __init__(self):
        super().__init__('isaac_visual_slam_node')
        
        # Initialize CvBridge
        self.cv_bridge = CvBridge()
        
        # TF buffer for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers for camera and IMU data
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_rect_color',
            self.rgb_callback,
            10
        )
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Publishers for SLAM output
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/visual_slam/odometry',
            10
        )
        
        self.map_publisher = self.create_publisher(
            Marker,
            '/visual_slam/map',
            10
        )
        
        # Internal state
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        self.imu_data = None
        self.point_cloud = []
        self.robot_pose = np.eye(4)  # 4x4 transformation matrix
        self.keyframe_poses = []
        
        # Feature detection parameters
        self.feature_detector = cv2.ORB_create(nfeatures=1000)
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Previous frame data for tracking
        self.prev_frame = None
        self.prev_features = None
        
        # Visualization
        self.br = CvBridge()
        
        self.get_logger().info('Isaac ROS Visual SLAM Node initialized')

    def rgb_callback(self, msg):
        """Process RGB camera data for feature detection"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect features in the current frame
            features = self.detect_features(cv_image)
            
            # If we have a previous frame, try to match features
            if self.prev_frame is not None and self.prev_features is not None:
                # Match features between current and previous frames
                matches = self.match_features(self.prev_frame, cv_image, self.prev_features, features)
                
                # Estimate motion using matched features
                transform = self.estimate_motion(matches, self.prev_features, features)
                
                # Update robot pose
                if transform is not None:
                    self.robot_pose = self.robot_pose @ transform
                    
                    # Publish odometry
                    self.publish_odometry(msg.header.stamp)
            
            # Store current frame for next iteration
            self.prev_frame = cv_image
            self.prev_features = features
            
            self.get_logger().info(f'Processed frame with {len(features[0]) if len(features) > 0 else 0} features')
            
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

    def depth_callback(self, msg):
        """Process depth data to build point cloud"""
        try:
            # Convert ROS Image to OpenCV
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Get camera parameters for 3D reconstruction
            if self.camera_info:
                # Reconstruct 3D points from depth and camera parameters
                points_3d = self.reconstruct_3d_points(depth_image, self.camera_info)
                
                # Add to map
                self.point_cloud.extend(points_3d)
                
                self.get_logger().info(f'Added {len(points_3d)} points to map')
        
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def camera_info_callback(self, msg):
        """Store camera information"""
        self.camera_info = msg

    def imu_callback(self, msg):
        """Process IMU data to constrain SLAM"""
        # In a real implementation, this would be used to constrain the SLAM solution
        # For now, we store the IMU data for potential fusion
        self.imu_data = msg

    def detect_features(self, image):
        """Detect features in the image using ORB"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        keypoints = self.feature_detector.detect(gray, None)
        keypoints, descriptors = self.feature_detector.compute(gray, keypoints)
        return keypoints, descriptors

    def match_features(self, prev_image, curr_image, prev_features, curr_features):
        """Match features between two images"""
        if len(prev_features[1]) == 0 or len(curr_features[1]) == 0:
            return []
        
        prev_desc = prev_features[1]
        curr_desc = curr_features[1]
        
        # Use brute force matcher
        matches = self.bf_matcher.match(prev_desc, curr_desc)
        
        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)
        
        # Keep only the best matches
        good_matches = matches[:int(len(matches) * 0.7)]
        
        return good_matches

    def estimate_motion(self, matches, prev_features, curr_features):
        """Estimate motion between frames using matched features"""
        if len(matches) < 10:
            return None
        
        # Get the coordinates of matched features
        prev_pts = np.float32([prev_features[0][m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        curr_pts = np.float32([curr_features[0][m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        
        # Compute essential matrix
        if self.camera_info:
            # Camera intrinsic parameters
            K = np.array([
                [self.camera_info.k[0], 0, self.camera_info.k[2]],
                [0, self.camera_info.k[4], self.camera_info.k[5]],
                [0, 0, 1]
            ])
            
            # Compute essential matrix
            E, mask = cv2.findEssentialMat(
                curr_pts, prev_pts, 
                cameraMatrix=K, 
                method=cv2.RANSAC, 
                prob=0.999, 
                threshold=1.0
            )
            
            # Recover pose
            if E is not None:
                _, R, t, _ = cv2.recoverPose(E, curr_pts, prev_pts, cameraMatrix=K)
                
                # Create transformation matrix
                transform = np.eye(4)
                transform[:3, :3] = R
                transform[:3, 3] = t.flatten()
                
                return transform
        
        return None

    def reconstruct_3d_points(self, depth_image, camera_info):
        """Reconstruct 3D points from depth image and camera parameters"""
        points_3d = []
        
        # Get camera intrinsic parameters
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        
        # Sample points from depth image
        height, width = depth_image.shape
        step = 10  # Sample every 10th pixel to reduce computation
        
        for y in range(0, height, step):
            for x in range(0, width, step):
                z = depth_image[y, x]
                
                # Skip invalid depth values
                if z > 0 and not np.isinf(z) and not np.isnan(z):
                    # Convert pixel coordinates to 3D world coordinates
                    X = (x - cx) * z / fx
                    Y = (y - cy) * z / fy
                    Z = z
                    
                    points_3d.append([X, Y, Z])
        
        return points_3d

    def publish_odometry(self, stamp):
        """Publish odometry based on estimated motion"""
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        
        # Extract position and orientation from pose matrix
        position = self.robot_pose[:3, 3]
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        
        # Convert rotation matrix to quaternion
        R = self.robot_pose[:3, :3]
        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2.0
        qx = (R[2,1] - R[1,2]) / (4 * qw)
        qy = (R[0,2] - R[2,0]) / (4 * qw)
        qz = (R[1,0] - R[0,1]) / (4 * qw)
        
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        
        # For now, just publish the pose (velocity estimation would require more complex tracking)
        self.odom_publisher.publish(odom_msg)

    def publish_map(self):
        """Publish the map as markers for visualization"""
        # This would create and publish markers representing the map
        # For simplicity, we'll just publish a single marker for the latest point
        if self.point_cloud:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "map_points"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Get the last point in the point cloud
            last_point = self.point_cloud[-1]
            marker.pose.position.x = last_point[0]
            marker.pose.position.y = last_point[1]
            marker.pose.position.z = last_point[2]
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1 
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            
            self.map_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    slam_node = IsaacVisualSLAMNode()

    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        slam_node.get_logger().info('SLAM node stopped by user')
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="isaac_nav" label="Isaac ROS Navigation">

```python
#!/usr/bin/env python3

"""
Isaac ROS Navigation Node for Humanoid Robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer
import numpy as np
import math


class IsaacNavigationNode(Node):

    def __init__(self):
        super().__init__('isaac_navigation_node')
        
        # TF buffer for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )
        
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.odom_subscription = self.create_subscription(
            PoseStamped,
            '/visual_slam/odometry',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.path_publisher = self.create_publisher(
            Path,
            '/navigation/global_plan',
            10
        )
        
        self.local_plan_publisher = self.create_publisher(
            Path,
            '/navigation/local_plan',
            10
        )
        
        # Internal state
        self.current_goal = None
        self.robot_pose = PoseStamped()
        self.laser_data = None
        self.path = []
        self.local_plan = []
        self.is_moving = False
        
        # Navigation parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.arrival_threshold = 0.2  # meters
        self.rotation_threshold = 0.1  # radians
        
        # Timer for navigation control
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)
        
        self.get_logger().info('Isaac ROS Navigation Node initialized')

    def goal_callback(self, msg):
        """Receive navigation goal"""
        self.current_goal = msg
        self.get_logger().info(f'Received navigation goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        
        # Plan path to goal
        self.plan_path_to_goal()
        
        # Set movement flag
        self.is_moving = True

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle detection"""
        self.laser_data = msg

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_pose = msg

    def plan_path_to_goal(self):
        """Simple path planning (in real implementation, use A* or other algorithm)"""
        if self.current_goal is None:
            return
        
        # For this example, we'll just create a straight-line path
        # In a real implementation, this would use a proper path planning algorithm
        start_x = self.robot_pose.pose.position.x
        start_y = self.robot_pose.pose.position.y
        
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        
        # Create waypoints along a straight line
        num_waypoints = 10
        self.path = []
        
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.position.z = 0.0
            
            # Set orientation toward next waypoint
            if i < num_waypoints:
                next_x = start_x + (i+1)/num_waypoints * (goal_x - start_x)
                next_y = start_y + (i+1)/num_waypoints * (goal_y - start_y)
                yaw = math.atan2(next_y - y, next_x - x)
                
                # Convert yaw to quaternion
                waypoint.pose.orientation.w = math.cos(yaw/2)
                waypoint.pose.orientation.z = math.sin(yaw/2)
            else:
                # At goal, maintain current orientation
                waypoint.pose.orientation = self.robot_pose.pose.orientation
            
            self.path.append(waypoint)
        
        # Publish path for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.poses = self.path
        self.path_publisher.publish(path_msg)
        
        self.get_logger().info(f'Planned path with {len(self.path)} waypoints')

    def navigation_loop(self):
        """Main navigation control loop"""
        if not self.is_moving or not self.current_goal:
            return
        
        if not self.robot_pose:
            return
        
        # Get robot position
        robot_x = self.robot_pose.pose.position.x
        robot_y = self.robot_pose.pose.position.y
        
        # Get goal position
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        
        # Calculate distance to goal
        dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        
        # Check if we've reached the goal
        if dist_to_goal < self.arrival_threshold:
            self.get_logger().info('Reached goal position')
            self.stop_robot()
            self.is_moving = False
            return
        
        # Calculate required rotation to face goal
        desired_yaw = math.atan2(goal_y - robot_y, goal_x - robot_x)
        current_yaw = self.get_robot_yaw()
        
        # Calculate angle difference
        angle_diff = desired_yaw - current_yaw
        # Normalize angle to [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create twist message for movement
        twist = Twist()
        
        # If robot is not facing the goal, rotate first
        if abs(angle_diff) > self.rotation_threshold:
            twist.angular.z = np.clip(angle_diff * 1.0, -self.angular_speed, self.angular_speed)
        else:
            # Move forward toward goal
            twist.linear.x = min(self.linear_speed, dist_to_goal * 1.0)  # Scale speed with distance
            twist.angular.z = 0.0
        
        # Publish command
        self.cmd_vel_publisher.publish(twist)
        
        # Check for obstacles in LIDAR data
        if self.laser_data:
            min_distance = min(self.laser_data.ranges)
            if min_distance < 0.5:  # Stop if obstacle closer than 0.5m
                self.get_logger().warn('Obstacle detected, stopping robot')
                self.stop_robot()
                # TODO: Implement obstacle avoidance behavior

    def get_robot_yaw(self):
        """Extract yaw from robot's orientation quaternion"""
        quat = self.robot_pose.pose.orientation
        # Convert quaternion to Euler angles (yaw is the only one we need)
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

    def get_local_plan(self):
        """Get a local plan for obstacle avoidance"""
        # In a real implementation, this would use local planner like DWA or TEB
        # For now, return the global plan as local
        return self.path


def main(args=None):
    rclpy.init(args=args)

    nav_node = IsaacNavigationNode()

    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        nav_node.get_logger().info('Navigation node stopped by user')
    finally:
        nav_node.stop_robot()
        nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="isaac_launch_file" label="Isaac ROS Launch File">

```python
#!/usr/bin/env python3

"""
Launch file for Isaac ROS Visual SLAM and Navigation stack
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace', default='humanoid_robot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_textbook_examples',
        executable='isaac_visual_slam',
        name='isaac_visual_slam_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/camera/rgb/image_rect_color', '/head_camera/image_raw'),
            ('/camera/depth/image_rect_raw', '/head_depth_camera/depth_image'),
            ('/imu/data', '/torso_imu'),
            ('/visual_slam/odometry', 'visual_odom')
        ],
        output='screen'
    )
    
    # Isaac ROS Navigation node
    navigation_node = Node(
        package='isaac_textbook_examples', 
        executable='isaac_navigation',
        name='isaac_navigation_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/head_lidar/scan'),
            ('/visual_slam/odometry', 'visual_odom'),
            ('/cmd_vel', 'cmd_vel')
        ],
        output='screen'
    )
    
    # Isaac ROS stereo image rectification (if using stereo cameras)
    stereo_rectify_node = Node(
        package='isaac_ros_stereo_image_proc',
        executable='isaac_ros_stereo_rectify',
        name='stereo_rectify_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Isaac ROS Visual Odometry (if using stereo)
    visual_odom_node = Node(
        package='isaac_ros_visual_odometry',
        executable='visual_odometry_node',
        name='visual_odom_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='humanoid_robot',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        visual_slam_node,
        navigation_node,
        # Note: Stereo nodes are included as examples.
        # The actual Isaac ROS packages may have different names and interfaces.
    ])
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [isaac_visual_slam_node]: Isaac ROS Visual SLAM Node initialized
[INFO] [1678882844.123456789] [isaac_navigation_node]: Isaac ROS Navigation Node initialized
[INFO] [1678882844.123456789] [isaac_visual_slam_node]: Processed frame with 523 features
[INFO] [1678882844.123456789] [isaac_visual_slam_node]: Added 45 points to map
[INFO] [1678882844.123456789] [isaac_navigation_node]: Received navigation goal: (2.50, 1.50)
[INFO] [1678882844.123456789] [isaac_navigation_node]: Planned path with 10 waypoints
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Path Planning**: Implement advanced path planning algorithms
   - [ ] Use A* or Dijkstra's algorithm for global planning
   - [ ] Implement a local planner for obstacle avoidance
   - [ ] Test navigation in complex environments
   - [ ] Evaluate path quality and computation time

2. **SLAM Optimization**: Improve the SLAM system performance
   - [ ] Implement loop closure detection
   - [ ] Add pose graph optimization
   - [ ] Optimize feature detection and matching
   - [ ] Evaluate mapping accuracy and efficiency

## Common Pitfalls and Solutions

- **Pitfall 1**: GPU memory limitations - Isaac ROS packages can be memory-intensive
  - *Solution*: Monitor GPU memory usage and optimize algorithms accordingly
- **Pitfall 2**: Integration complexity - Connecting Isaac ROS with custom systems
  - *Solution*: Start with provided examples and gradually add custom functionality
- **Pitfall 3**: Timing issues - SLAM requires precise timing between sensors
  - *Solution*: Use message filters for proper synchronization
- **Pitfall 4**: Calibration requirements - Cameras and IMU need proper calibration
  - *Solution*: Follow calibration procedures in Isaac Sim and ROS 2

## Summary

- Isaac ROS provides GPU-accelerated packages for robotics
- Visual SLAM enables localization and mapping using cameras
- Navigation systems plan and execute robot motion
- Proper integration with Isaac Sim enhances realism
- GPU acceleration significantly improves performance

## Further Reading

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages.html)
- [Visual SLAM in Robotics](https://ieeexplore.ieee.org/document/9414029)
- [Navigation and Path Planning](https://navigation.ros.org/)
- [GPU-Accelerated Robotics](https://arxiv.org/abs/2109.07503)