---
title: "ROS 2 Nodes Deep Dive"
description: "Understanding nodes, lifecycle, and parameter management"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Create complex ROS 2 nodes with parameter management
- Understand and implement node lifecycle management
- Apply advanced logging and debugging techniques in ROS 2
- Design nodes that follow ROS 2 best practices

## Introduction

In ROS 2, a node is the fundamental building block of any robot application. It represents a single executable that performs computation and communicates with other nodes in the system. Understanding nodes deeply is crucial for developing robust robotic systems that can operate reliably in dynamic environments like those encountered with humanoid robots in Physical AI applications.

This chapter delves deeper into node concepts, exploring how to create nodes that follow best practices for parameter management, logging, and lifecycle management. We'll also examine advanced techniques for debugging and managing complex node interactions in real-world scenarios.

## Core Concepts

ROS 2 nodes are designed to be more robust and flexible than their ROS 1 counterparts. They include built-in features for parameters, logging, and lifecycle management that make them suitable for production environments. Understanding these features is essential for creating high-quality robotics applications.

### Node Parameters

Parameters in ROS 2 provide a way to configure node behavior without recompiling. They can be set at launch time, programmatically, or through command-line arguments. Parameters support several data types and can be declared with default values and constraints to ensure robust configuration.

### Node Lifecycle

ROS 2 includes an optional lifecycle system that provides explicit state transitions for nodes. This is particularly useful for safety-critical applications where knowing the exact state of a node is important. The lifecycle system defines states like unconfigured, inactive, active, and finalized, with transitions between them.

### Logging and Debugging

ROS 2 provides comprehensive logging capabilities through the rcl_logging interface. Nodes can use different logging levels (DEBUG, INFO, WARN, ERROR, FATAL) and these messages can be filtered and formatted according to system needs. This is essential for debugging complex robotic systems.

## Hands-on Examples

Let's create a more advanced ROS 2 node with parameters and lifecycle management:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="advanced_node" label="Advanced Node" default>

```python
#!/usr/bin/env python3

"""
Advanced ROS 2 node with parameters and lifecycle management
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class AdvancedNode(Node):

    def __init__(self):
        super().__init__('advanced_node')
        
        # Declare parameters with default values
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('sensor_frequency', 10.0)
        self.declare_parameter('max_distance', 5.0)
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.sensor_frequency = self.get_parameter('sensor_frequency').value
        self.max_distance = self.get_parameter('max_distance').value
        
        # Log parameter values
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Sensor frequency: {self.sensor_frequency} Hz')
        self.get_logger().info(f'Max distance: {self.max_distance} meters')
        
        # Create a publisher with QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher_ = self.create_publisher(String, 'robot_status', qos_profile)
        
        # Create a timer
        self.timer = self.create_timer(1.0 / self.sensor_frequency, self.timer_callback)
        
        # Counter for messages
        self.i = 0
        
        # Log initialization
        self.get_logger().info(f'Advanced node initialized for {self.robot_name}')

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.robot_name} status report: {self.i}'
        self.publisher_.publish(msg)
        if self.i % 10 == 0:  # Log every 10th message
            self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    advanced_node = AdvancedNode()

    try:
        rclpy.spin(advanced_node)
    except KeyboardInterrupt:
        advanced_node.get_logger().info('Node interrupted by user')
    finally:
        advanced_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="parameter_client" label="Parameter Client">

```python
#!/usr/bin/env python3

"""
Parameter client example to set parameters on the advanced node
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor


class ParameterClient(Node):

    def __init__(self):
        super().__init__('parameter_client')
        # This node will call services to set parameters on another node
        self.target_node_name = 'advanced_node'
        self.timer = self.create_timer(5.0, self.change_parameters)

    def change_parameters(self):
        # In a real implementation, you would call the set_parameters service
        # of the target node here
        self.get_logger().info(f'Changing parameters on {self.target_node_name}')
        # Set new values for parameters
        # This would typically be done via the set_parameters service
        # Example:
        # client = self.create_client(SetParameters, f'/{self.target_node_name}/set_parameters')
        # request = SetParameters.Request()
        # request.parameters = [Parameter('sensor_frequency', Parameter.Type.DOUBLE, 15.0)]
        # future = client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    
    param_client = ParameterClient()
    
    try:
        # Use MultiThreadedExecutor to handle callbacks properly
        executor = MultiThreadedExecutor()
        executor.add_node(param_client)
        executor.spin()
    except KeyboardInterrupt:
        param_client.get_logger().info('Parameter client interrupted by user')
    finally:
        param_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [advanced_node]: Robot name: humanoid_robot
[INFO] [1678882844.123456789] [advanced_node]: Sensor frequency: 10.0 Hz
[INFO] [1678882844.123456789] [advanced_node]: Max distance: 5.0 meters
[INFO] [1678882844.123456789] [advanced_node]: Advanced node initialized for humanoid_robot
[INFO] [1678882854.123456789] [advanced_node]: Published: "humanoid_robot status report: 10"
[INFO] [1678882864.123456789] [advanced_node]: Published: "humanoid_robot status report: 20"
...
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Parameter Management**: Create a node with multiple parameters of different types
   - [ ] Declare parameters of type integer, double, string, and boolean
   - [ ] Implement logic that changes behavior based on parameter values
   - [ ] Test changing parameters at runtime using command line tools
   - [ ] Document how parameters affect your node's behavior

2. **Logging Strategy**: Implement comprehensive logging in a custom node
   - [ ] Use different log levels (debug, info, warn, error) appropriately
   - [ ] Create meaningful log messages that aid debugging
   - [ ] Test log filtering by level
   - [ ] Set up log file output for the node

## Common Pitfalls and Solutions

- **Pitfall 1**: Not declaring parameters properly - Using get_parameter without declaring first can cause runtime errors
  - *Solution*: Always declare parameters with declare_parameter() before using get_parameter()
- **Pitfall 2**: Forgetting to handle parameter changes - Parameters can change during runtime, nodes should be resilient
  - *Solution*: Implement parameter callbacks using add_on_set_parameters_callback()
- **Pitfall 3**: Poor logging practices - Too much or too little logging makes debugging difficult
  - *Solution*: Use appropriate log levels and provide context in log messages
- **Pitfall 4**: Memory management in nodes - Creating timers, publishers, subscribers without proper cleanup
  - *Solution*: Always properly destroy ROS objects in the destroy_node() method

## Summary

- Nodes in ROS 2 can have parameters that configure their behavior without recompilation
- The lifecycle system provides explicit state management for safety-critical applications
- Comprehensive logging is essential for debugging complex robotic systems
- Proper parameter declaration and management is crucial for robust nodes
- Memory management and proper cleanup are important for node lifecycle

## Further Reading

- [ROS 2 Parameters Guide](https://docs.ros.org/en/humble/How-To-Guides/Parameters-in-CPP.html)
- [Node Lifecycle Documentation](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS 2 Logging System](https://docs.ros.org/en/humble/How-To-Guides/Logging.html)
- [QoS Configuration](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)