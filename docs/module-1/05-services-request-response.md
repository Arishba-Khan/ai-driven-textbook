---
title: "Services: Request-Response Communication"
description: "Implementing services for synchronous communication in ROS 2"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Create custom service definitions for robotic applications
- Implement service servers that handle requests robustly
- Build clients that properly communicate with services
- Design service-based architectures for humanoid robot control

## Introduction

Services in ROS 2 provide a request-response communication pattern that is synchronous in nature, contrasting with the asynchronous publish-subscribe pattern of topics. In humanoid robotics, services are essential for operations that require guaranteed delivery and a direct response, such as configuration changes, calibration procedures, or state queries that must be completed before proceeding.

The synchronous nature of services makes them ideal for scenarios where a node must receive a specific response to a request before continuing its operation. This is particularly important in Physical AI applications where precise control and state verification are required for safe robot operation.

## Core Concepts

ROS 2 services follow a client-server model where a client sends a request to a server and waits for a response. Services are defined with a specific interface containing both request and response message types. This interface must be known by both the client and server for successful communication.

### Service Definition

Service definitions are specified in `.srv` files that describe both the request and response message structures. The request section defines inputs to the service, and the response section defines the output.

### Service Server

A service server implements the business logic for handling requests. When a client sends a request, the server's callback function is executed, processes the request, and returns a response.

### Service Client

A service client sends requests to a server and waits for the response. Clients can be synchronous (blocking) or asynchronous (non-blocking), depending on the application requirements.

### Use Cases

Services are appropriate for operations that:
- Require guaranteed delivery and response
- Have a clear request-response pattern
- Need to block execution until completion
- Are infrequently called or time-insensitive

## Hands-on Examples

Let's implement custom services for humanoid robot control:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="service_def" label="Service Definition" default>

```python
# Service file: SetJointPositions.srv
# Save as srv/SetJointPositions.srv

# Request - joint names and target positions
string[] joint_names
float64[] positions

---
# Response - success status and error message
bool success
string message
```

</TabItem>
<TabItem value="service_server" label="Service Server">

```python
#!/usr/bin/env python3

"""
Humanoid robot service server for joint position control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import time
from threading import Lock

# Import or define your service
# from humanoid_robot_msgs.srv import SetJointPositions


class JointControlService(Node):

    def __init__(self):
        super().__init__('joint_control_server')
        
        # Create a lock for thread safety
        self.lock = Lock()
        
        # Store current joint positions (simulated hardware interface)
        self.joint_positions = {}
        self.default_joints = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist'
        ]
        
        # Initialize to zero
        for joint in self.default_joints:
            self.joint_positions[joint] = 0.0
        
        # Create service server
        self.srv = self.create_service(
            # SetJointPositions,  # Replace with actual service type
            'std_srvs/srv/Empty',  # Using Empty service as placeholder
            'set_joint_positions',
            self.set_joint_positions_callback
        )
        
        # Create a publisher for joint states to broadcast new positions
        from sensor_msgs.msg import JointState
        self.joint_publisher = self.create_publisher(
            JointState,
            'joint_states',
            QoSProfile(depth=10)
        )
        
        self.get_logger().info('Joint control service server initialized')

    def set_joint_positions_callback(self, request, response):
        # In a real implementation, we would use the actual request/response types
        # For this example, we'll simulate the functionality
        
        with self.lock:
            try:
                # Simulate setting joint positions
                # In a real system, this would interface with actual hardware
                
                # Validate request
                if len(request.joint_names) != len(request.positions):
                    response.success = False
                    response.message = "joint_names and positions arrays must have the same length"
                    return response
                
                # Update stored joint positions
                for joint_name, position in zip(request.joint_names, request.positions):
                    if joint_name in self.joint_positions:
                        self.joint_positions[joint_name] = position
                    else:
                        # Add new joint if needed
                        self.joint_positions[joint_name] = position
                
                # Simulate time for the movement to complete
                time.sleep(0.05)  # 50ms movement time
                
                # Publish updated joint states
                self.publish_joint_states()
                
                response.success = True
                response.message = f"Successfully set {len(request.joint_names)} joint positions"
                
                self.get_logger().info(response.message)
                
            except Exception as e:
                response.success = False
                response.message = f"Error setting joint positions: {str(e)}"
                self.get_logger().error(response.message)
        
        return response

    def publish_joint_states(self):
        """Publish current joint states to the joint_states topic"""
        from sensor_msgs.msg import JointState
        from builtin_interfaces.msg import Time
        
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'
        
        # Populate joint names and positions
        joint_msg.name = list(self.joint_positions.keys())
        joint_msg.position = list(self.joint_positions.values())
        
        # Set default velocities and efforts
        joint_msg.velocity = [0.0] * len(joint_msg.position)
        joint_msg.effort = [0.0] * len(joint_msg.position)
        
        self.joint_publisher.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)

    joint_control_server = JointControlService()

    try:
        rclpy.spin(joint_control_server)
    except KeyboardInterrupt:
        joint_control_server.get_logger().info('Service server stopped by user')
    finally:
        joint_control_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="service_client" label="Service Client">

```python
#!/usr/bin/env python3

"""
Service client for humanoid robot control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import time
import sys

# Import or define your service
# from humanoid_robot_msgs.srv import SetJointPositions


class JointControlClient(Node):

    def __init__(self):
        super().__init__('joint_control_client')
        
        # Create client for the joint position service
        self.cli = self.create_client(
            # SetJointPositions,  # Replace with actual service type
            'std_srvs/srv/Empty',  # Using Empty service as placeholder
            'set_joint_positions'
        )
        
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Connected to joint control service')

    def send_request(self, joint_names, positions):
        # Create request object (placeholder implementation)
        # In a real implementation, we would use SetJointPositions.Request()
        request = type('Request', (), {})()
        request.joint_names = joint_names
        request.positions = positions
        
        # Send async request
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    
    joint_control_client = JointControlClient()
    
    # Example: Move humanoid to a specific pose
    joint_names = [
        'left_hip', 'left_knee', 'left_ankle',
        'right_hip', 'right_knee', 'right_ankle',
        'left_shoulder', 'left_elbow', 'left_wrist',
        'right_shoulder', 'right_elbow', 'right_wrist'
    ]
    
    # Define a standing pose (all zeros)
    positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    try:
        # Send the request
        response = joint_control_client.send_request(joint_names, positions)
        
        if response and response.success:
            print(f'Success: {response.message}')
        else:
            print(f'Error: {response.message if response else "No response"}')
            
    except KeyboardInterrupt:
        joint_control_client.get_logger().info('Client interrupted by user')
    finally:
        joint_control_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [joint_control_server]: Joint control service server initialized
[INFO] [1678882844.123456789] [joint_control_client]: Connected to joint control service
Success: Successfully set 12 joint positions
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Custom Service Creation**: Create a service for querying robot state
   - [ ] Define a custom service interface for querying robot pose
   - [ ] Implement a service server that responds with current robot state
   - [ ] Create a client that requests and processes the robot state
   - [ ] Test the service with different query parameters

2. **Service Reliability**: Implement a service with timeout and retry logic
   - [ ] Add timeout handling to the client when calling services
   - [ ] Implement retry logic for failed service calls
   - [ ] Create a circuit breaker pattern to handle service unavailability
   - [ ] Add appropriate logging for service call outcomes

## Common Pitfalls and Solutions

- **Pitfall 1**: Blocking the main thread - Synchronous service calls blocking node execution
  - *Solution*: Use asynchronous service calls with proper future handling
- **Pitfall 2**: No timeout handling - Clients hanging indefinitely if service is unavailable
  - *Solution*: Always implement timeout mechanisms and error handling for service calls
- **Pitfall 3**: Incompatible service interfaces - Client and server using different service definitions
  - *Solution*: Ensure both client and server use the same service interface version
- **Pitfall 4**: Thread safety issues - Multiple threads accessing service resources simultaneously
  - *Solution*: Use locks or other synchronization primitives where needed

## Summary

- Services provide synchronous request-response communication in ROS 2
- Custom service definitions specify request and response message structures
- Service servers implement the business logic for handling requests
- Service clients send requests and receive responses from servers
- Services are appropriate for operations requiring guaranteed delivery and response

## Further Reading

- [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Services/Understanding-ROS2-Services.html)
- [Creating Custom Services](https://docs.ros.org/en/humble/How-To-Guides/Creating-Your-First-ROS2-Service.html)
- [Service Implementation Best Practices](https://docs.ros.org/en/humble/How-To-Guides/Using-Client-Libraries.html)
- [Asynchronous Service Calls](https://docs.ros.org/en/humble/How-To-Guides/Using-Async-Sync-Spinner.html)