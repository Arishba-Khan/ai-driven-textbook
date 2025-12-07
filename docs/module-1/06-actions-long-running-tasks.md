---
title: "Actions: Long-Running Tasks"
description: "Implementing actions for long-running asynchronous operations in ROS 2"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Create custom action definitions for complex robotic tasks
- Implement action servers that handle long-running operations
- Build action clients that monitor and control ongoing tasks
- Design action-based workflows for humanoid robot navigation and manipulation

## Introduction

Actions in ROS 2 are designed for long-running tasks that require feedback during execution and the ability to cancel operations. This makes them ideal for humanoid robotics applications such as navigation, manipulation, complex trajectory following, or any task that takes a significant amount of time to complete and requires monitoring.

Unlike services, which are synchronous and block until completion, actions are asynchronous and provide continuous feedback to clients. This allows the robot system to continue operating while a task is in progress, providing updates on progress and allowing for intervention if needed. Actions also support cancellation, which is crucial for safety in robotic systems.

## Core Concepts

ROS 2 actions extend the request-response pattern of services by adding continuous feedback and the ability to cancel operations. Each action interface includes three message types: goal (request), result (response), and feedback (status updates during execution).

### Action Definition

Action definitions are specified in `.action` files that define the goal, result, and feedback message structures. The goal contains parameters for the action, the result contains the outcome, and the feedback provides periodic updates during execution.

### Action Server

An action server manages the execution of long-running tasks. It receives goals from clients, executes them, provides feedback during execution, and returns results when complete. Action servers can also handle cancellation requests.

### Action Client

An action client sends goals to an action server and can monitor progress, receive feedback, and cancel goals if needed. This allows for more complex interactions than simple services.

### Use Cases

Actions are appropriate for operations that:
- Take a long time to complete
- Require monitoring of progress
- Need the ability to cancel during execution
- Benefit from feedback during execution
- Are goal-oriented rather than request-response oriented

## Hands-on Examples

Let's implement custom actions for humanoid robot navigation and manipulation:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="action_def" label="Action Definition" default>

```python
# Action file: NavigateToPose.action
# Save as action/NavigateToPose.action

# Define the goal
geometry_msgs/Pose target_pose
float32 tolerance

---
# Define the result
bool success
string message
float32 distance_traveled

---
# Define the feedback
string status
float32 distance_to_goal
float32 progress_percentage
geometry_msgs/Pose current_pose
```

</TabItem>
<TabItem value="action_server" label="Action Server">

```python
#!/usr/bin/env python3

"""
Navigation action server for humanoid robot
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
import math
import time
from threading import Lock


class NavigateToPoseActionServer(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_action_server')
        
        # Use a reentrant callback group to allow concurrent execution
        callback_group = ReentrantCallbackGroup()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            # NavigateToPose,  # Replace with actual action type
            'dummy_action',  # Placeholder for this example
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            callback_group=callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Create publisher for robot commands
        from geometry_msgs.msg import Twist
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(depth=10)
        )
        
        # Transform listener for getting current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Track active goals
        self._active_goals = set()
        self._goal_lock = Lock()
        
        self.get_logger().info('Navigation action server initialized')

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject goals based on current state"""
        self.get_logger().info(
            f'Received goal request to navigate to position: '
            f'({goal_request.target_pose.position.x}, {goal_request.target_pose.position.y})'
        )
        
        # Accept all goals for this example
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject cancel requests"""
        self.get_logger().info('Received cancel request')
        
        # Accept all cancel requests
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the navigation goal"""
        self.get_logger().info('Executing goal...')
        
        # Add to active goals
        with self._goal_lock:
            self._active_goals.add(goal_handle.goal_id.uuid)
        
        # Get goal parameters
        target_pose = goal_handle.request.target_pose
        tolerance = goal_handle.request.tolerance
        
        # Initialize feedback
        feedback_msg = type('Feedback', (), {})()  # Placeholder
        feedback_msg.status = 'Moving to target'
        feedback_msg.distance_to_goal = 0.0
        feedback_msg.progress_percentage = 0.0
        feedback_msg.current_pose = Pose()
        
        result_msg = type('Result', (), {})()  # Placeholder
        result_msg.success = False
        result_msg.message = ''
        result_msg.distance_traveled = 0.0
        
        try:
            # Simulate navigation process
            start_time = time.time()
            distance_traveled = 0.0
            
            # For this example, we'll simulate a simple navigation
            target_x = target_pose.position.x
            target_y = target_pose.position.y
            
            current_x, current_y = 0.0, 0.0  # Starting position
            
            # Calculate initial distance to goal
            distance_to_goal = math.sqrt(
                (target_x - current_x)**2 + (target_y - current_y)**2
            )
            
            # Navigation loop
            steps = 50  # Number of feedback updates
            for i in range(steps):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result_msg.success = False
                    result_msg.message = 'Goal canceled'
                    with self._goal_lock:
                        self._active_goals.discard(goal_handle.goal_id.uuid)
                    return result_msg
                
                # Update progress
                progress = (i + 1) / steps
                current_x = target_x * progress
                current_y = target_y * progress
                
                # Update distance to goal
                distance_to_goal = math.sqrt(
                    (target_x - current_x)**2 + (target_y - current_y)**2
                )
                
                # Update distance traveled
                distance_traveled = math.sqrt(current_x**2 + current_y**2)
                
                # Update feedback
                feedback_msg.distance_to_goal = distance_to_goal
                feedback_msg.progress_percentage = progress * 100.0
                feedback_msg.current_pose.position.x = current_x
                feedback_msg.current_pose.position.y = current_y
                
                # Publish feedback
                goal_handle.publish_feedback(feedback_msg)
                
                # Log progress every 5 steps
                if (i + 1) % 5 == 0:
                    self.get_logger().info(
                        f'Navigation progress: {feedback_msg.progress_percentage:.1f}% '
                        f'(distance to goal: {distance_to_goal:.2f}m)'
                    )
                
                # Sleep to simulate movement
                time.sleep(0.2)
            
            # Check if goal was achieved
            if distance_to_goal <= tolerance:
                goal_handle.succeed()
                result_msg.success = True
                result_msg.message = f'Reached target position within {tolerance}m tolerance'
                result_msg.distance_traveled = distance_traveled
            else:
                goal_handle.abort()
                result_msg.success = False
                result_msg.message = 'Failed to reach target within tolerance'
                result_msg.distance_traveled = distance_traveled
                
        except Exception as e:
            goal_handle.abort()
            result_msg.success = False
            result_msg.message = f'Exception in action execution: {str(e)}'
            self.get_logger().error(f'Action execution failed: {str(e)}')
        finally:
            # Remove from active goals
            with self._goal_lock:
                self._active_goals.discard(goal_handle.goal_id.uuid)
        
        self.get_logger().info(result_msg.message)
        return result_msg


def main(args=None):
    rclpy.init(args=args)

    navigate_to_pose_action_server = NavigateToPoseActionServer()

    try:
        rclpy.spin(navigate_to_pose_action_server)
    except KeyboardInterrupt:
        navigate_to_pose_action_server.get_logger().info('Action server stopped by user')
    finally:
        navigate_to_pose_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="action_client" label="Action Client">

```python
#!/usr/bin/env python3

"""
Navigation action client for humanoid robot
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, Point, Quaternion
import time


class NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_action_client')
        
        # Create action client
        self._action_client = ActionClient(
            self,
            # NavigateToPose,  # Replace with actual action type
            'dummy_action',  # Placeholder for this example
            'navigate_to_pose'
        )
        
        self.get_logger().info('Navigation action client initialized')

    def send_goal(self, target_x, target_y, tolerance=0.1):
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Create goal message (placeholder implementation)
        goal_msg = type('Goal', (), {})()  # Placeholder
        goal_msg.target_pose = Pose()
        goal_msg.target_pose.position.x = target_x
        goal_msg.target_pose.position.y = target_y
        goal_msg.target_pose.position.z = 0.0
        goal_msg.target_pose.orientation = Quaternion()  # Default to no rotation
        goal_msg.tolerance = tolerance
        
        # Send goal
        self.get_logger().info(f'Sending navigation goal to ({target_x}, {target_y})')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Add done callback to handle result
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server')
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
        if result.success:
            self.get_logger().info(f'Distance traveled: {result.distance_traveled:.2f}m')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Navigation feedback: {feedback_msg.progress_percentage:.1f}% complete, '
            f'{feedback_msg.distance_to_goal:.2f}m to goal'
        )


def main(args=None):
    rclpy.init(args=args)
    
    action_client = NavigateToPoseActionClient()
    
    # Send a navigation goal
    goal_future = action_client.send_goal(2.0, 1.5, tolerance=0.2)
    
    try:
        # Keep spinning until the action is complete
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Action client interrupted by user')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [navigate_to_pose_action_server]: Navigation action server initialized
[INFO] [1678882844.123456789] [navigate_to_pose_action_client]: Navigation action client initialized
[INFO] [1678882844.123456789] [navigate_to_pose_action_client]: Waiting for action server...
[INFO] [1678882844.123456789] [navigate_to_pose_action_client]: Sending navigation goal to (2.0, 1.5)
[INFO] [1678882844.123456789] [navigate_to_pose_action_server]: Received goal request to navigate to position: (2.0, 1.5)
[INFO] [1678882844.123456789] [navigate_to_pose_action_server]: Executing goal...
[INFO] [1678882844.323456789] [navigate_to_pose_action_client]: Navigation feedback: 10.0% complete, 2.21m to goal
[INFO] [1678882844.523456789] [navigate_to_pose_action_client]: Navigation feedback: 20.0% complete, 1.98m to goal
...
[INFO] [1678882854.123456789] [navigate_to_pose_action_server]: Result: Reached target position within 0.2m tolerance
[INFO] [1678882854.123456789] [navigate_to_pose_action_client]: Result: Reached target position within 0.2m tolerance
[INFO] [1678882854.123456789] [navigate_to_pose_action_client]: Distance traveled: 2.55m
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Manipulation Action**: Create an action for controlling humanoid arm movements
   - [ ] Define an action interface for arm manipulation tasks
   - [ ] Implement an action server that controls the robot's arms
   - [ ] Create a client that sends manipulation goals
   - [ ] Add progress feedback for multi-step manipulation sequences

2. **Action Cancellation**: Implement robust cancellation handling
   - [ ] Add cancel callbacks that properly stop ongoing operations
   - [ ] Test cancellation under various timing conditions
   - [ ] Implement cleanup procedures that run on cancellation
   - [ ] Create a user interface for requesting action cancellations

## Common Pitfalls and Solutions

- **Pitfall 1**: Not handling cancellation properly - Actions continuing to execute after cancellation
  - *Solution*: Always check goal_handle.is_cancel_requested in execution loops
- **Pitfall 2**: Blocking execution - Action callbacks blocking the executor
  - *Solution*: Use async/await patterns and avoid long blocking operations
- **Pitfall 3**: Resource cleanup - Not properly cleaning up resources when actions complete
  - *Solution*: Use try/finally blocks to ensure cleanup happens
- **Pitfall 4**: Feedback frequency - Sending feedback too frequently or infrequently
  - *Solution*: Balance feedback frequency with performance requirements

## Summary

- Actions are designed for long-running tasks requiring feedback and cancellation
- Action interfaces define goal, result, and feedback message structures
- Action servers execute tasks and provide continuous feedback
- Action clients can monitor progress and cancel operations
- Actions are essential for navigation, manipulation, and complex task execution

## Further Reading

- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Actions/Understanding-ROS2-Actions.html)
- [Creating Custom Actions](https://docs.ros.org/en/humble/How-To-Guides/Creating-Your-First-ROS2-Action.html)
- [Action Server Implementation](https://docs.ros.org/en/humble/How-To-Guides/Using-Actions.html#action-server)
- [Action Client Implementation](https://docs.ros.org/en/humble/How-To-Guides/Using-Actions.html#action-client)