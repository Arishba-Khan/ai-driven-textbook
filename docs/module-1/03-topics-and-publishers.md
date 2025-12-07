---
title: "Topics and Publishers"
description: "Understanding publish-subscribe communication in ROS 2"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Implement publishers for custom message types in ROS 2
- Understand Quality of Service (QoS) settings and their impact
- Design effective message structures for humanoid robot applications
- Create robust topic-based communication patterns

## Introduction

Topics form the backbone of communication in ROS 2, implementing a publish-subscribe pattern that enables loose coupling between nodes. In the context of Physical AI and humanoid robotics, topics are essential for distributing sensor data, robot states, and control commands throughout the system. Understanding how to design and implement effective topic-based communication is crucial for creating responsive and reliable robotic systems.

The publish-subscribe pattern allows multiple nodes to broadcast information (publishers) and consume information (subscribers) without direct knowledge of each other. This architecture is particularly advantageous in complex robotic systems where different components need to share data without tight coupling, enabling modularity and maintainability.

## Core Concepts

ROS 2 topics use a Data Distribution Service (DDS) implementation at their core, which provides more sophisticated communication patterns than ROS 1. This includes configurable Quality of Service (QoS) settings that allow fine-tuning of communication characteristics based on requirements like reliability, latency, and bandwidth.

### Message Types

Messages are the data structures exchanged between nodes via topics. ROS 2 supports built-in message types as well as custom message definitions. For humanoid robotics applications, messages might represent sensor readings, robot poses, joint states, or command velocities.

### Quality of Service (QoS)

QoS settings allow publishers and subscribers to negotiate communication characteristics. These include:

- Reliability: Best effort vs. reliable delivery
- Durability: Volatile vs. transient local
- History: Keep all or keep last N messages
- Deadline: Delivery deadline for messages
- Lifespan: How long messages are valid

### Publisher-Subscriber Pattern

The pattern involves publishers sending messages to topics and subscribers receiving messages from topics. Multiple publishers and subscribers can interact with the same topic, creating a flexible communication network.

## Hands-on Examples

Let's implement custom messages and QoS patterns for humanoid robot communication:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="custom_msg" label="Custom Message" default>

```python
# Custom message file: HumanoidState.msg
# Save as msg/HumanoidState.msg

# Humanoid robot state message
string robot_name
float64 torso_height
float64[3] center_of_mass  # x, y, z
float64[28] joint_positions  # 28 DOF humanoid
float64[28] joint_velocities
float64[28] joint_efforts
float64[4] orientation  # quaternion
float64[3] linear_velocity
float64[3] angular_velocity
bool in_motion
bool in_safe_mode
```

</TabItem>
<TabItem value="publisher" label="State Publisher">

```python
#!/usr/bin/env python3

"""
Humanoid robot state publisher
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header

# Import your custom message
# from humanoid_robot_msgs.msg import HumanoidState


class HumanoidStatePublisher(Node):

    def __init__(self):
        super().__init__('humanoid_state_publisher')
        
        # Configure QoS for real-time robot state data
        # High reliability and small history for real-time data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only keep the most recent message
        )
        
        # Publish joint states
        self.joint_publisher = self.create_publisher(
            JointState, 
            'joint_states', 
            qos_profile
        )
        
        # Publish robot state (using built-in message as example)
        self.robot_state_publisher = self.create_publisher(
            JointState,  # Using JointState as placeholder for custom message
            'humanoid_robot_state',
            qos_profile
        )
        
        # Timer for publishing at 50Hz (every 20ms)
        self.timer = self.create_timer(0.02, self.publish_robot_state)
        
        self.i = 0

    def publish_robot_state(self):
        # Create joint state message
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'
        
        # Simulate humanoid joint names and positions
        joint_names = [
            'left_hip', 'left_knee', 'left_ankle', 
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist'
            # ... add more as needed
        ]
        joint_msg.name = joint_names
        
        # Simulate joint positions (in radians)
        positions = []
        for i, name in enumerate(joint_names):
            # Generate simulated position data
            pos = 0.1 * self.i * (i + 1) % (2 * 3.14159)
            positions.append(pos)
        joint_msg.position = positions
        
        # Simulate velocities and efforts
        velocities = [0.1] * len(joint_names)  # Simulated velocities
        efforts = [0.5] * len(joint_names)     # Simulated efforts
        joint_msg.velocity = velocities
        joint_msg.effort = efforts
        
        # Publish the joint state
        self.joint_publisher.publish(joint_msg)
        
        # Publish to robot state topic (with custom message, this would be different)
        self.robot_state_publisher.publish(joint_msg)
        
        # Log every 50th message
        if self.i % 50 == 0:
            self.get_logger().info(f'Published robot state: {self.i}')
        
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    state_publisher = HumanoidStatePublisher()

    try:
        rclpy.spin(state_publisher)
    except KeyboardInterrupt:
        state_publisher.get_logger().info('State publisher stopped by user')
    finally:
        state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="qos_example" label="QoS Configuration">

```python
#!/usr/bin/env python3

"""
QoS configuration examples for different types of data
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


def get_sensor_qos():
    """QoS for sensor data - best effort with small history"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5  # Keep last 5 sensor readings
    )


def get_control_qos():
    """QoS for control commands - reliable with small history"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=3  # Keep last 3 commands (in case of resend)
    )


def get_configuration_qos():
    """QoS for configuration data - reliable with larger history"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_ALL,
        depth=10  # Keep all configuration changes
    )


def get_feedback_qos():
    """QoS for feedback data - reliable with moderate history"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10  # Keep last 10 feedback messages
    )


# Example usage
def create_publishers_example(node):
    """Example of creating publishers with appropriate QoS"""
    
    # Sensor data publisher (e.g., IMU, cameras)
    sensor_pub = node.create_publisher(
        String,  # Replace with actual sensor message type
        'sensor_data',
        get_sensor_qos()
    )
    
    # Control command publisher (e.g., joint commands)
    control_pub = node.create_publisher(
        String,  # Replace with actual control message type
        'control_commands',
        get_control_qos()
    )
    
    # Configuration publisher
    config_pub = node.create_publisher(
        String,  # Replace with actual config message type
        'configuration',
        get_configuration_qos()
    )
    
    # Feedback publisher (e.g., achievement of goals)
    feedback_pub = node.create_publisher(
        String,  # Replace with actual feedback message type
        'feedback',
        get_feedback_qos()
    )
    
    return sensor_pub, control_pub, config_pub, feedback_pub


if __name__ == '__main__':
    print("QoS Configuration Examples")
    print("Sensor Data:", get_sensor_qos())
    print("Control Commands:", get_control_qos())
    print("Configuration:", get_configuration_qos())
    print("Feedback:", get_feedback_qos())
```

</TabItem>
</Tabs>

Expected Output:
```
QoS Configuration Examples
Sensor Data: QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=5)
Control Commands: QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=3)
Configuration: QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_ALL, depth=10)
Feedback: QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Custom Message Creation**: Create and use a custom message for humanoid robot sensor data
   - [ ] Define a custom .msg file for sensor readings
   - [ ] Generate the message files with colcon build
   - [ ] Implement a publisher that sends sensor data
   - [ ] Create a subscriber that processes the sensor data

2. **QoS Optimization**: Experiment with different QoS settings for various data types
   - [ ] Set up publishers with different QoS profiles
   - [ ] Measure the impact on latency and reliability
   - [ ] Document which QoS settings work best for different robot data types
   - [ ] Create a configuration table for different message types

## Common Pitfalls and Solutions

- **Pitfall 1**: Using inappropriate QoS settings - Applying reliable QoS to high-frequency sensor data causing network congestion
  - *Solution*: Use best effort QoS for high-frequency sensor data; reliable QoS for critical commands
- **Pitfall 2**: Message type mismatch - Publishers and subscribers using incompatible message structures
  - *Solution*: Ensure both ends use the same message definition and version
- **Pitfall 3**: Insufficient history depth - Setting depth=1 for important messages that may have processing delays
  - *Solution*: Set appropriate history depth based on message importance and system requirements
- **Pitfall 4**: Publishing too frequently - Overwhelming the network with unnecessary data
  - *Solution*: Optimize publishing frequency based on actual system needs

## Summary

- Topics enable publish-subscribe communication in ROS 2 using DDS
- QoS settings allow customization of communication characteristics
- Custom messages are essential for domain-specific robot applications
- Proper QoS selection is crucial for system performance and reliability
- Message design affects system modularity and maintainability

## Further Reading

- [ROS 2 Topics and Services](https://docs.ros.org/en/humble/Tutorials/Topics/Using-DataTypes.html)
- [Quality of Service in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [Message Definition Guide](https://docs.ros.org/en/humble/How-To-Guides/Defining-custom-interfaces.html)
- [ROS 2 Communication Patterns](https://design.ros2.org/articles/topic_and_service_composition.html)