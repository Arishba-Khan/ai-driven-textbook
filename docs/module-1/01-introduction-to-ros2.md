---
title: "Introduction to ROS 2"
description: "Getting started with Robot Operating System 2"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the purpose and architecture of Robot Operating System 2 (ROS 2)
- Identify the key differences between ROS 1 and ROS 2
- Create a basic ROS 2 workspace and understand its structure

## Introduction

Robot Operating System 2 (ROS 2) is not actually an operating system, but rather a collection of tools, libraries, and conventions that aim to simplify the development of complex and robust robot applications. It provides a framework for writing robot software, handling communication between processes, and managing the complexity that arises in robot development.

ROS 2 is the next generation of the Robot Operating System, addressing limitations in the original ROS 1, particularly in the areas of real-time performance, multi-robot systems, and commercial product deployment. It features a modern architecture built on DDS (Data Distribution Service) for communication, which provides improved support for real-time systems and multi-robot scenarios.

ROS 2 is essential for robotics development because it provides standardized tools and approaches that are widely used in both research and industry, enabling reproducible results and code sharing across the robotics community. Understanding ROS 2 is fundamental for working with humanoid robots in the context of Physical AI.

## Core Concepts

ROS 2 represents a significant evolution from ROS 1, built from the ground up to address the needs of commercial and industrial robotics applications. Unlike ROS 1, which relied on a centralized master node, ROS 2 uses a decentralized architecture that allows for more robust multi-robot systems and better integration with existing distributed systems.

### ROS 2 Architecture

The core architecture of ROS 2 is built around a DDS (Data Distribution Service) implementation. DDS is an open standard for machine-to-machine communication that provides a middleware for distributed systems. This architecture provides:

- **Decentralized communication**: No single point of failure
- **Real-time capabilities**: Support for time-sensitive applications
- **Multi-robot support**: Native handling of multiple robots in the same network
- **Language neutrality**: Support for multiple programming languages beyond C++ and Python

### Key Components

ROS 2 consists of several key components that work together to provide a comprehensive development environment:

- **Nodes**: Basic computational units that perform computation. In ROS 2, nodes are designed to be more robust and can exist in multiple processes.
- **Topics**: Named buses over which nodes exchange messages. Topics implement a publish-subscribe communication pattern.
- **Services**: Synchronous request-response communication between nodes.
- **Actions**: Asynchronous communication pattern for long-running tasks with feedback.

### Ecosystem and Tools

ROS 2 provides a rich ecosystem of tools and libraries:

- **RCL**: Robot Client Libraries provide the core ROS 2 functionality in different languages
- **RMW**: ROS Middleware Interface abstracts the underlying DDS implementation
- **Launch**: System for starting multiple nodes at once
- **ROS Bags**: Tools for recording and playing back data

## Hands-on Examples

Let's create a basic ROS 2 workspace and a simple publisher node:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="workspace" label="Create Workspace" default>

```bash
# Create a new ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source the ROS 2 installation
source /opt/ros/humble/setup.bash  # Adjust for your ROS 2 distribution

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

</TabItem>
<TabItem value="publisher" label="Publisher Node">

```python
#!/usr/bin/env python3

"""
Basic ROS 2 publisher node example
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="package" label="Package.xml">

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>demo_nodes_py</name>
  <version>0.0.0</version>
  <description>Examples of minimal publishers using Python</description>
  <maintainer email="todo@todo.todo">todo</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1678882844.623456789] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1678882845.123456789] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Basic Workspace**: Create a new ROS 2 workspace following the setup instructions
   - [ ] Install ROS 2 on your system
   - [ ] Create a workspace directory structure
   - [ ] Build and source the workspace
   - [ ] Verify the installation works correctly

2. **Publisher Node**: Create and run the publisher node example
   - [ ] Create a new package for your examples
   - [ ] Implement the publisher node code
   - [ ] Build and run the node
   - [ ] Verify it's publishing messages to the topic

## Common Pitfalls and Solutions

- **Pitfall 1**: Forgetting to source the ROS 2 installation - Many users forget to source /opt/ros/humble/setup.bash before working
  - *Solution*: Always source your ROS 2 environment and workspace before running commands
- **Pitfall 2**: Workspace path confusion - Not understanding the difference between the installation path and your workspace path
  - *Solution*: Remember that /opt/ros/humble is where ROS 2 is installed, while ~/ros2_ws is your personal workspace
- **Pitfall 3**: Python path issues - Python modules not being found when running ROS 2 nodes
  - *Solution*: After building with colcon, always source the setup file in your workspace's install directory

## Summary

- ROS 2 is a framework for robot application development built on DDS for improved reliability
- The architecture is decentralized, making it suitable for multi-robot systems
- Key components include nodes, topics, services, and actions for communication
- Creating a workspace is the first step to developing ROS 2 applications
- Basic publisher nodes demonstrate the fundamental publish-subscribe pattern

## Further Reading

- [ROS 2 Documentation](https://docs.ros.org/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [DDS Primer](https://www.omg.org/omg-dds-portal/)
- [Designing ROS 2 Packages](https://design.ros2.org/articles/package_description.html)