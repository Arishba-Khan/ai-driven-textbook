---
title: "URDF for Humanoid Robots"
description: "Creating robot descriptions using URDF in ROS 2"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Create comprehensive URDF files for humanoid robots
- Understand the structure and elements of URDF robot descriptions
- Implement accurate kinematic chains for humanoid robot models
- Integrate URDF models with ROS 2 for visualization and simulation

## Introduction

URDF (Unified Robot Description Format) is the standard XML format for representing robot models in ROS. For humanoid robots in Physical AI applications, URDF is essential for defining the robot's physical structure, kinematic chains, and visual properties. A properly configured URDF model enables simulation, visualization, motion planning, and control algorithms to work effectively with the robot.

Understanding URDF is crucial for robotics development as it provides the foundation for how the robot is represented in simulation and how planning algorithms understand the robot's structure. In humanoid robotics, with their complex multi-degree-of-freedom systems, URDF becomes even more important for managing the complexity of the robot's kinematic structure.

## Core Concepts

URDF uses an XML-based format to describe robot models with a tree-like structure of links connected by joints. Each link represents a rigid body with properties such as mass, inertia, visual geometry, and collision geometry. Joints define the connection between links and specify the type of motion allowed between them.

### Links

Links represent rigid bodies in the robot structure. Each link can have multiple properties including:
- Visual: Defines how the link looks in visualizations
- Collision: Defines the collision geometry used in physics simulation
- Inertial: Defines the mass, center of mass, and inertia tensor

### Joints

Joints connect links and define the degrees of freedom between them. Common joint types include:
- Fixed: No movement allowed between links
- Revolute: Rotational movement around a single axis
- Continuous: Rotational movement without limits
- Prismatic: Linear sliding movement
- Floating: Six degrees of freedom (rarely used in practical robots)

### Robots with Multiple Chains

Humanoid robots have multiple kinematic chains (arms, legs) that share a common torso, making their URDF more complex than simpler robots. Managing these multiple chains requires careful attention to the tree structure.

## Hands-on Examples

Let's create a URDF model for a humanoid robot:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="urdf_example" label="Basic Humanoid URDF" default>

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 .1 .2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 .8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 .1 .2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
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

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
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

  <!-- Left Leg -->
  <joint name="torso_to_left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_hip"/>
    <origin xyz="-0.08 0 -0.15"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_hip">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Continue defining other joints and links for legs, arms, etc. -->
  
</robot>
```

</TabItem>
<TabItem value="urdf_launch" label="URDF Launch File">

```python
#!/usr/bin/env python3

"""
Launch file to load and visualize URDF in ROS 2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    urdf_model_path = LaunchConfiguration('model')
    urdf_model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([FindPackageShare('my_robot_description'), 'urdf', 'simple_humanoid.urdf']),
        description='URDF model file path'
    )

    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': True,
            'rate': 30,
        }]
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'publish_frequency': 30.0,
            'use_tf_static': True,
        }],
        arguments=[urdf_model_path]
    )

    # RViz2 node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('my_robot_description'), 'rviz', 'urdf_viewer.rviz'])]
    )

    return LaunchDescription([
        urdf_model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
```

</TabItem>
<TabItem value="urdf_ros_node" label="URDF Integration Node">

```python
#!/usr/bin/env python3

"""
Node to work with URDF model in ROS 2
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class URDFIntegrationNode(Node):

    def __init__(self):
        super().__init__('urdf_integration_node')
        
        # Declare parameters
        self.declare_parameter(
            'robot_description', 
            '',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Robot URDF description'
            )
        )
        
        # Transform broadcaster for publishing TFs
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing transforms
        self.timer = self.create_timer(0.1, self.publish_transforms)
        
        # Initialize joint angles for animation
        self.joint_positions = {
            'torso_to_head': 0.0,
            'torso_to_left_hip': 0.0,
            # Add more joints as needed
        }
        
        self.time_counter = 0.0
        
        self.get_logger().info('URDF integration node initialized')

    def publish_transforms(self):
        """Publish transforms for the robot's joints"""
        self.time_counter += 0.01
        
        # Animate some joints for demonstration
        animated_joints = {
            'torso_to_head': math.sin(self.time_counter) * 0.3,
            'torso_to_left_hip': math.cos(self.time_counter) * 0.2
        }
        
        for joint_name, position in animated_joints.items():
            self.joint_positions[joint_name] = position
        
        # Create and publish transforms for each joint
        transforms = []
        
        # Head joint transform
        head_transform = TransformStamped()
        head_transform.header.stamp = self.get_clock().now().to_msg()
        head_transform.header.frame_id = 'torso'
        head_transform.child_frame_id = 'head'
        head_transform.transform.translation.x = 0.0
        head_transform.transform.translation.y = 0.0
        head_transform.transform.translation.z = 0.35
        # Simple rotation around Y axis for the head
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, self.joint_positions['torso_to_head'], 0)
        head_transform.transform.rotation.x = q[0]
        head_transform.transform.rotation.y = q[1]
        head_transform.transform.rotation.z = q[2]
        head_transform.transform.rotation.w = q[3]
        transforms.append(head_transform)
        
        # Left hip joint transform
        hip_transform = TransformStamped()
        hip_transform.header.stamp = self.get_clock().now().to_msg()
        hip_transform.header.frame_id = 'torso'
        hip_transform.child_frame_id = 'left_hip'
        hip_transform.transform.translation.x = -0.08
        hip_transform.transform.translation.y = 0.0
        hip_transform.transform.translation.z = -0.15
        # Simple rotation around Z axis for the hip
        q = quaternion_from_euler(0, 0, self.joint_positions['torso_to_left_hip'])
        hip_transform.transform.rotation.x = q[0]
        hip_transform.transform.rotation.y = q[1]
        hip_transform.transform.rotation.z = q[2]
        hip_transform.transform.rotation.w = q[3]
        transforms.append(hip_transform)
        
        # Publish all transforms
        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    urdf_integration_node = URDFIntegrationNode()

    try:
        rclpy.spin(urdf_integration_node)
    except KeyboardInterrupt:
        urdf_integration_node.get_logger().info('URDF integration node stopped by user')
    finally:
        urdf_integration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
</Tabs>

Expected Output:
```
The URDF will be visualized in RViz2 showing the humanoid robot model with animated joints.
The robot_state_publisher will publish the robot's state to the /tf and /tf_static topics.
The joint_state_publisher will allow manual control of joint positions.
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Complete Robot Model**: Extend the basic URDF to include all humanoid joints
   - [ ] Add both arms with shoulder, elbow, and wrist joints
   - [ ] Complete both legs with hip, knee, and ankle joints
   - [ ] Add appropriate visual and collision geometry for each link
   - [ ] Test the complete model in RViz2

2. **Kinematic Analysis**: Implement forward kinematics for the humanoid model
   - [ ] Write a node that computes forward kinematics
   - [ ] Calculate end-effector positions for hands and feet
   - [ ] Visualize the end-effector positions in RViz2
   - [ ] Verify the kinematic chain is correctly defined

## Common Pitfalls and Solutions

- **Pitfall 1**: Incorrect mass properties - Setting unrealistic mass and inertia values
  - *Solution*: Use proper CAD software to calculate accurate mass properties or approximate based on geometric shapes
- **Pitfall 2**: Undefined parent-child relationships - Creating disconnected kinematic chains
  - *Solution*: Ensure every link (except the base) has exactly one parent and is connected via a joint
- **Pitfall 3**: Wrong joint limits - Setting limits that don't match physical robot capabilities
  - *Solution*: Research the actual hardware specifications or use conservative estimates for simulation
- **Pitfall 4**: Collision mesh issues - Using high-polygon meshes that slow down simulation
  - *Solution*: Use simplified collision geometry with fewer polygons than visual geometry

## Summary

- URDF is the standard format for robot description in ROS
- A URDF model defines links (rigid bodies) connected by joints
- Humanoid robots require complex URDF structures with multiple kinematic chains
- Proper URDF models are essential for simulation, visualization, and control
- Integration with ROS 2 allows real-time visualization and control of the robot model

## Further Reading

- [URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)
- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [Robot State Publisher](https://docs.ros.org/en/humble/p/robot_state_publisher/)
- [TF2 Transform Library](https://docs.ros.org/en/humble/p/tf2/)
- [Gazebo Integration](https://classic.gazebosim.org/tutorials?cat=connect_ros)