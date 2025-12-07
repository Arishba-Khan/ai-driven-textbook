---
title: "Introduction to Gazebo"
description: "Getting started with Gazebo simulation for robotics"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the Gazebo simulation environment and its role in robotics
- Set up and configure a basic Gazebo simulation
- Integrate Gazebo with ROS 2 for robotics simulation
- Create and modify simple simulation worlds

## Introduction

Gazebo is a powerful, open-source robotics simulator that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It plays a critical role in the Physical AI & Humanoid Robotics pipeline by allowing developers to test algorithms, validate robot designs, and train AI models in a safe, controlled environment before deploying to real hardware.

Simulation is an essential component of modern robotics development, enabling rapid prototyping, testing, and validation without the risks and costs associated with physical hardware. Gazebo's realistic physics engine and extensive model library make it particularly suitable for humanoid robotics, where safety and cost considerations are paramount during development.

## Core Concepts

Gazebo operates as a standalone simulation engine but integrates seamlessly with ROS 2 through the `gazebo_ros_pkgs`. This integration enables bidirectional communication between the simulation and ROS 2 nodes, allowing robots to operate in simulation as if they were running on real hardware. The simulator models physics, sensors, and environments with high fidelity, making it an invaluable tool for developing and testing robotic systems.

### Simulation Architecture

Gazebo's architecture consists of:
- **Physics Engine**: ODE, Bullet, or DART for realistic physics simulation
- **Rendering Engine**: OpenSceneGraph for 3D visualization
- **Sensor Simulation**: Models for cameras, lidars, IMUs, and other sensors
- **Model Database**: A library of pre-built robot and environment models
- **Plugin System**: Extensible architecture for custom simulation components

### ROS 2 Integration

The `gazebo_ros_pkgs` package provides essential interfaces between Gazebo and ROS 2, including:
- Publishers for sensor data (cameras, lidars, etc.)
- Services for simulation control (reset, pause, etc.)
- TF broadcasters for robot pose information
- Controllers for joint actuation

### World Building

Gazebo worlds are defined using SDF (Simulation Description Format), an XML-based format that describes the environment, objects, lighting, and physics properties. Users can create custom worlds or use existing models from the Gazebo Model Database.

## Hands-on Examples

Let's set up a basic Gazebo simulation integrated with ROS 2:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="launch_file" label="Gazebo Launch File" default>

```xml
<?xml version="1.0"?>
<launch>
  <!-- Arguments -->
  <arg name="world" default="empty.sdf"/>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- Gazebo server -->
  <node name="gazebo" pkg="gazebo_ros" exec="gzserver" respawn="false"
        output="screen" args="-u $(var paused) $(var world)">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Gazebo client -->
  <node name="gazebo_gui" pkg="gazebo_ros" exec="gzclient" respawn="false"
        output="screen" args="" if="$(var gui)">
  </node>
  
  <!-- Spawn robot in simulation -->
  <node name="spawn_entity" pkg="gazebo_ros" exec="spawn_entity.py"
        output="screen"
        args="-topic robot_description
              -entity simple_humanoid
              -x 0.0 -y 0.0 -z 1.0">
  </node>
</launch>
```

</TabItem>
<TabItem value="robot_spawn" label="Robot Spawn Node">

```python
#!/usr/bin/env python3

"""
Node to spawn a robot in Gazebo and control it via ROS 2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import time


class GazeboRobotInterface(Node):

    def __init__(self):
        super().__init__('gazebo_robot_interface')
        
        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/simple_humanoid/cmd_vel',
            10
        )
        
        # Subscriber for Gazebo model states
        self.model_states_subscriber = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        
        # Timer to send commands periodically
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0
        
        self.get_logger().info('Gazebo Robot Interface initialized')

    def model_states_callback(self, msg):
        """Receive model states from Gazebo"""
        for i, name in enumerate(msg.name):
            if name == 'simple_humanoid':
                position = msg.pose[i].position
                self.get_logger().info(
                    f'Robot position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})'
                )
                break

    def timer_callback(self):
        """Send movement commands to the robot"""
        twist = Twist()
        
        # Move in a square pattern
        if self.i < 50:  # Move forward
            twist.linear.x = 0.5
        elif self.i < 100:  # Turn right
            twist.angular.z = -0.5
        elif self.i < 150:  # Move forward
            twist.linear.x = 0.5
        elif self.i < 200:  # Turn right
            twist.angular.z = -0.5
        else:
            self.i = 0
        
        self.cmd_vel_publisher.publish(twist)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    gazebo_robot_interface = GazeboRobotInterface()

    try:
        rclpy.spin(gazebo_robot_interface)
    except KeyboardInterrupt:
        gazebo_robot_interface.get_logger().info('Node stopped by user')
    finally:
        gazebo_robot_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="simple_world" label="Simple World SDF">

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Add a simple box obstacle -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.5 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1667</iyy>
            <iyz>0</iyz>
            <izz>0.1667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Add a simple room structure -->
    <model name="room">
      <static>true</static>
      <link name="wall1">
        <pose>0 -5 1.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.2 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.2 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
      <link name="wall2">
        <pose>0 5 1.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.2 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.2 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [gazebo_robot_interface]: Gazebo Robot Interface initialized
[INFO] [1678882844.123456789] [gazebo_robot_interface]: Robot position: (0.00, 0.00, 1.00)
[INFO] [1678882844.223456789] [gazebo_robot_interface]: Robot position: (0.05, 0.00, 1.00)
[INFO] [1678882844.323456789] [gazebo_robot_interface]: Robot position: (0.10, 0.00, 1.00)
...
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Custom Environment**: Create a custom Gazebo world with obstacles
   - [ ] Design a world file with multiple obstacles
   - [ ] Add textured surfaces to make it more realistic
   - [ ] Test your robot's navigation in the new environment
   - [ ] Document how world complexity affects simulation performance

2. **Sensor Integration**: Add sensors to your simulated robot
   - [ ] Integrate a camera sensor in the simulation
   - [ ] Publish camera data to a ROS 2 topic
   - [ ] Create a simple image processing node to consume the data
   - [ ] Test that the sensor data is realistic and usable

## Common Pitfalls and Solutions

- **Pitfall 1**: Simulation instability - Physics simulation becoming unstable with certain parameters
  - *Solution*: Tune physics parameters, reduce time steps, or simplify collision meshes
- **Pitfall 2**: Performance degradation - Simulation running slowly with complex worlds
  - *Solution*: Optimize collision meshes, reduce model complexity, or upgrade hardware
- **Pitfall 3**: Inaccurate sensor simulation - Sensor data not matching real-world behavior
  - *Solution*: Fine-tune sensor parameters and consider environmental factors
- **Pitfall 4**: Model import issues - Robot models not loading correctly in Gazebo
  - *Solution*: Verify URDF to SDF conversion, check joint and link definitions

## Summary

- Gazebo provides realistic physics simulation for robotics development
- ROS 2 integration enables seamless transition between simulation and real hardware
- World building uses SDF format to define environments and objects
- Simulation is essential for safe, cost-effective robot development
- Proper physics and sensor modeling is critical for sim-to-real transfer

## Further Reading

- [Gazebo Tutorial](https://classic.gazebosim.org/tutorials)
- [ROS 2 with Gazebo](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [SDF Specification](http://sdformat.org/spec)
- [Physics Engine Comparison](https://gazebosim.org/docs/fortress/physics/)