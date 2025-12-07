---
title: "Sensor Simulation: Lidar, Depth, IMU"
description: "Simulating sensors in Gazebo for robotics perception"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Configure and integrate simulated sensors in Gazebo
- Set up lidar, depth camera, and IMU sensors for humanoid robots
- Process simulated sensor data in ROS 2
- Validate sensor accuracy and performance in simulation

## Introduction

Sensor simulation is critical for developing and testing perception systems in robotics, allowing developers to create robust algorithms without the constraints and risks of physical hardware. In Physical AI & Humanoid Robotics applications, simulated sensors must closely match their real-world counterparts to ensure that algorithms trained in simulation can transfer effectively to physical robots.

Gazebo provides high-quality simulation of various sensor types, including lidar, depth cameras, and IMUs, which are essential for humanoid robot perception. These sensors enable robots to understand their environment, localize themselves, and interact with objects. Properly configured sensor simulation allows for the development of perception algorithms that can later be deployed on real hardware with minimal adjustments.

## Core Concepts

Simulated sensors in Gazebo operate by generating synthetic data based on the virtual environment, mimicking the behavior of real sensors. This involves ray tracing for lidar and depth sensors, and mathematical models for IMUs and other inertial sensors.

### Sensor Types in Gazebo

Gazebo supports a wide range of sensor types:
- **Ray/Lidar Sensors**: Simulate time-of-flight sensors with configurable resolution and range
- **Depth Cameras**: Generate depth maps in addition to RGB images
- **IMU Sensors**: Simulate accelerometers and gyroscopes with noise models
- **GPS Sensors**: Provide position estimates in world coordinates
- **Force/Torque Sensors**: Measure forces and torques at joints

### Sensor Integration with ROS 2

The `gazebo_ros_pkgs` provide plugins to bridge Gazebo sensors with ROS 2 topics, allowing simulated sensor data to be processed by the same ROS 2 nodes that would handle real sensor data. This seamless integration is key to the sim-to-real transfer approach.

### Noise Modeling

Real sensors include various sources of noise and inaccuracies. Gazebo allows modeling of sensor noise, bias, and drift, making the simulation more realistic and improving the robustness of algorithms developed in simulation.

## Hands-on Examples

Let's implement sensor simulation in Gazebo:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="sensor_robot" label="Robot with Sensors URDF" default>

```xml
<?xml version="1.0"?>
<robot name="sensor_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head with sensors -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Sensors defined in the URDF using Gazebo plugins -->
  
  <!-- IMU sensor in the torso -->
  <gazebo reference="torso">
    <sensor name="torso_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

  <!-- Depth camera in the head -->
  <gazebo reference="head">
    <sensor name="head_depth_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
        <image>
          <format>R8G8B8</format>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>head</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>

  <!-- 360-degree lidar on the head -->
  <gazebo reference="head">
    <sensor name="head_lidar" type="ray">
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle> <!-- -π radians -->
            <max_angle>3.14159</max_angle>   <!-- π radians -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
        <frame_name>head</frame_name>
        <topic_name>scan</topic_name>
      </plugin>
      <always_on>true</always_on>
      <update_rate>10</update_rate>
    </sensor>
  </gazebo>

  <!-- Front-facing RGB camera -->
  <gazebo reference="head">
    <sensor name="head_camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <format>R8G8B8</format>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="rgb_camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>head</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/sensor_humanoid</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

</TabItem>
<TabItem value="sensor_processing" label="Sensor Processing Node">

```python
#!/usr/bin/env python3

"""
Node to process simulated sensor data from Gazebo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped


class SensorProcessingNode(Node):

    def __init__(self):
        super().__init__('sensor_processing_node')
        
        # Initialize CvBridge for image processing
        self.cv_bridge = CvBridge()
        
        # TF buffer for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers for different sensor types
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/sensor_humanoid/scan',
            self.lidar_callback,
            10
        )
        
        self.camera_subscription = self.create_subscription(
            Image,
            '/sensor_humanoid/head_camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/sensor_humanoid/head_depth_camera/depth_image',
            self.depth_callback,
            10
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/sensor_humanoid/torso_imu',
            self.imu_callback,
            10
        )
        
        # Publishers for processed data
        self.obstacle_publisher = self.create_publisher(
            PointCloud2,
            '/sensor_humanoid/obstacles',
            10
        )
        
        self.processed_image_publisher = self.create_publisher(
            Image,
            '/sensor_humanoid/processed_image',
            10
        )
        
        # Internal state
        self.latest_range = None
        self.latest_image = None
        self.camera_info = None
        self.imu_data = None
        
        self.get_logger().info('Sensor Processing Node initialized')

    def lidar_callback(self, msg):
        """Process lidar scan data"""
        # Process the scan to detect obstacles
        ranges = np.array(msg.ranges)
        
        # Filter out invalid ranges (inf, NaN)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            min_range = np.min(valid_ranges)
            self.get_logger().info(f'Lidar: Closest obstacle at {min_range:.2f}m')
            
            # Detect obstacles closer than 1m
            obstacle_indices = np.where(ranges < 1.0)[0]
            if len(obstacle_indices) > 0:
                self.get_logger().info(f'Lidar: {len(obstacle_indices)} obstacles detected within 1m')

    def camera_callback(self, msg):
        """Process RGB camera data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform basic image processing
            gray = cv2.cvtColor(cv_image, 'bgr8', cv2.COLOR_BGR2GRAY)
            
            # Detect edges using Canny
            edges = cv2.Canny(gray, 50, 150)
            
            # Convert back to ROS Image and publish
            processed_msg = self.cv_bridge.cv2_to_imgmsg(edges, encoding='mono8')
            processed_msg.header = msg.header
            self.processed_image_publisher.publish(processed_msg)
            
            self.get_logger().info(f'Processed camera image: {cv_image.shape[1]}x{cv_image.shape[0]}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def depth_callback(self, msg):
        """Process depth camera data"""
        try:
            # Convert depth image to numpy array
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Calculate depth statistics
            valid_depths = depth_image[np.isfinite(depth_image)]
            
            if len(valid_depths) > 0:
                avg_depth = np.mean(valid_depths)
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                
                self.get_logger().info(f'Depth: avg={avg_depth:.2f}m, min={min_depth:.2f}m, max={max_depth:.2f}m')
                
                # Detect objects at different distances
                close_objects = len(depth_image[depth_image < 1.0])  # Objects within 1m
                mid_objects = len(depth_image[(depth_image >= 1.0) & (depth_image < 3.0)])  # Objects 1-3m
                far_objects = len(depth_image[depth_image >= 3.0])  # Objects beyond 3m
                
                self.get_logger().info(f'Depth: {close_objects} close, {mid_objects} mid, {far_objects} far objects')
        
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract orientation
        orientation = msg.orientation
        # Extract angular velocity
        angular_velocity = msg.angular_velocity
        # Extract linear acceleration
        linear_acceleration = msg.linear_acceleration
        
        # Calculate roll, pitch, yaw from quaternion
        import math
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info(f'IMU: Roll={roll:.3f}, Pitch={pitch:.3f}, Yaw={yaw:.3f}')
        
        # Store for potential fusion with other sensors
        self.imu_data = {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration
        }

    def detect_obstacles_in_depth(self, depth_image, camera_info):
        """Detect obstacles in depth image using camera info"""
        # This would implement more sophisticated obstacle detection
        # using the depth data and camera parameters
        pass


def main(args=None):
    rclpy.init(args=args)

    sensor_processing_node = SensorProcessingNode()

    try:
        rclpy.spin(sensor_processing_node)
    except KeyboardInterrupt:
        sensor_processing_node.get_logger().info('Sensor processing node stopped by user')
    finally:
        sensor_processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="sensor_fusion" label="Sensor Fusion Node">

```python
#!/usr/bin/env python3

"""
Node to fuse data from multiple sensors
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PointStamped, Pose, Twist
from tf2_ros import TransformListener, Buffer
import numpy as np
import math


class SensorFusionNode(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers for different sensors
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/sensor_humanoid/scan',
            self.lidar_callback,
            10
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/sensor_humanoid/torso_imu',
            self.imu_callback,
            10
        )
        
        # Publisher for fused state
        self.state_publisher = self.create_publisher(
            Pose,
            '/sensor_humanoid/fused_state',
            10
        )
        
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/sensor_humanoid/fused_velocity',
            10
        )
        
        # Internal state
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.last_update_time = self.get_clock().now()
        
        # IMU bias compensation
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.bias_samples = []
        self.max_bias_samples = 100
        
        self.get_logger().info('Sensor Fusion Node initialized')

    def lidar_callback(self, msg):
        """Process lidar data for obstacle detection"""
        # Convert lidar scan to obstacle map
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        valid_ranges = []
        valid_angles = []
        
        for i, r in enumerate(msg.ranges):
            if np.isfinite(r) and msg.range_min <= r <= msg.range_max:
                valid_ranges.append(r)
                valid_angles.append(angles[i])
        
        if len(valid_ranges) > 0:
            # Calculate nearest obstacle direction
            min_idx = np.argmin(valid_ranges)
            nearest_angle = valid_angles[min_idx]
            nearest_distance = valid_ranges[min_idx]
            
            # Convert to Cartesian coordinates relative to robot
            x = nearest_distance * math.cos(nearest_angle)
            y = nearest_distance * math.sin(nearest_angle)
            
            self.get_logger().info(f'Nearest obstacle: {nearest_distance:.2f}m at ({x:.2f}, {y:.2f})')
            
            # Could implement more sophisticated mapping here
            self.publish_fused_state()

    def imu_callback(self, msg):
        """Process IMU data for orientation and motion estimation"""
        # Extract orientation quaternion
        orientation = np.array([
            msg.orientation.x,
            msg.orientation.y, 
            msg.orientation.z,
            msg.orientation.w
        ])
        
        # Extract angular velocity
        angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Extract linear acceleration
        linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # Update state estimate using IMU data
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time
        
        if dt > 0:
            # Integrate angular velocity to update orientation
            # This is a simplified integration - in practice, would use more sophisticated methods
            angular_speed = np.linalg.norm(angular_velocity)
            
            if angular_speed > 0.001:  # Avoid numerical issues
                axis = angular_velocity / angular_speed
                angle = angular_speed * dt
                
                # Convert axis-angle to quaternion
                s = math.sin(angle / 2)
                c = math.cos(angle / 2)
                
                dq = np.array([axis[0] * s, axis[1] * s, axis[2] * s, c])
                
                # Multiply quaternions to integrate rotation
                new_orientation = self.q_multiply(orientation, dq)
                
                # Normalize to prevent drift
                self.orientation = new_orientation / np.linalg.norm(new_orientation)
                
                # Update velocity using linear acceleration
                # Remove gravity from acceleration measurements
                gravity = np.array([0, 0, 9.81])
                
                # Rotate gravity vector to robot frame
                gravity_robot_frame = self.rotate_vector_by_quaternion(gravity, self.orientation)
                
                # Subtract gravity from measured acceleration
                net_acceleration = linear_acceleration - gravity_robot_frame
                
                # Integrate to get velocity
                self.velocity += net_acceleration * dt
                
                # Integrate velocity to get position
                self.position += self.velocity * dt
        
        # Publish updated state
        self.publish_fused_state()

    def q_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return np.array([w, x, y, z])

    def rotate_vector_by_quaternion(self, v, q):
        """Rotate a 3D vector using a quaternion"""
        # Convert vector to quaternion
        v_q = np.array([v[0], v[1], v[2], 0])
        
        # Conjugate of rotation quaternion
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        
        # Rotate: q * v * q_conj
        temp = self.q_multiply(q, v_q)
        rotated = self.q_multiply(temp, q_conj)
        
        return rotated[:3]

    def publish_fused_state(self):
        """Publish the fused state estimate"""
        pose_msg = Pose()
        
        # Position
        pose_msg.position.x = self.position[0]
        pose_msg.position.y = self.position[1]
        pose_msg.position.z = self.position[2]
        
        # Orientation
        pose_msg.orientation.x = self.orientation[0]
        pose_msg.orientation.y = self.orientation[1]
        pose_msg.orientation.z = self.orientation[2]
        pose_msg.orientation.w = self.orientation[3]
        
        self.state_publisher.publish(pose_msg)
        
        # Publish velocity
        twist_msg = Twist()
        twist_msg.linear.x = self.velocity[0]
        twist_msg.linear.y = self.velocity[1]
        twist_msg.linear.z = self.velocity[2]
        
        self.velocity_publisher.publish(twist_msg)
        
        self.get_logger().info(f'Fused State: Pos ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}), '
                              f'Vel ({self.velocity[0]:.2f}, {self.velocity[1]:.2f}, {self.velocity[2]:.2f})')


def main(args=None):
    rclpy.init(args=args)

    sensor_fusion_node = SensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        sensor_fusion_node.get_logger().info('Sensor fusion node stopped by user')
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [sensor_processing_node]: Sensor Processing Node initialized
[INFO] [1678882844.123456789] [sensor_processing_node]: Processed camera image: 640x480
[INFO] [1678882844.123456789] [sensor_processing_node]: Depth: avg=2.45m, min=0.15m, max=9.80m
[INFO] [1678882844.123456789] [sensor_processing_node]: Depth: 12 close, 45 mid, 200 far objects
[INFO] [1678882844.123456789] [sensor_processing_node]: IMU: Roll=0.012, Pitch=-0.008, Yaw=0.003
[INFO] [1678882844.123456789] [sensor_processing_node]: Lidar: Closest obstacle at 1.25m
[INFO] [1678882844.123456789] [sensor_fusion_node]: Sensor Fusion Node initialized
[INFO] [1678882844.123456789] [sensor_fusion_node]: Fused State: Pos (0.00, 0.00, 0.00), Vel (0.00, 0.00, 0.00)
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Sensor Fusion**: Implement a more sophisticated sensor fusion algorithm
   - [ ] Combine lidar and IMU data for improved position estimation
   - [ ] Implement a Kalman filter for sensor fusion
   - [ ] Test how the fused data improves navigation performance
   - [ ] Compare fused estimates to ground truth

2. **Obstacle Detection**: Develop a comprehensive obstacle detection system
   - [ ] Use multiple sensors to detect and map obstacles
   - [ ] Create a navigation costmap from sensor data
   - [ ] Implement path planning based on sensor data
   - [ ] Validate the system in complex environments

## Common Pitfalls and Solutions

- **Pitfall 1**: Noise in sensor data - Raw sensor data contains noise that affects processing
  - *Solution*: Implement filtering and outlier detection algorithms
- **Pitfall 2**: Sensor synchronization - Data from different sensors arrives at different times
  - *Solution*: Implement time synchronization and interpolation techniques
- **Pitfall 3**: Coordinate frame mismatches - Sensors in different frame of references
  - *Solution*: Use TF transforms to properly align sensor data
- **Pitfall 4**: Performance issues - Processing multiple sensors in real-time
  - *Solution*: Optimize algorithms and reduce sensor update rates where appropriate

## Summary

- Sensor simulation enables development of perception algorithms without physical hardware
- Gazebo supports various sensor types with realistic models
- ROS 2 integration allows seamless processing of simulated sensor data
- Sensor fusion combines data from multiple sources for improved estimates
- Proper noise modeling ensures algorithms work in real scenarios

## Further Reading

- [Gazebo Sensors Documentation](http://gazebosim.org/tutorials?tut=ros_gz_sensors)
- [Sensor Processing with ROS 2](https://navigation.ros.org/tutorials/docs/get_back_to_home.html)
- [IMU Integration Techniques](https://www.mdpi.com/1424-8220/19/7/1670)
- [Sensor Fusion in Robotics](https://ieeexplore.ieee.org/document/8953217)