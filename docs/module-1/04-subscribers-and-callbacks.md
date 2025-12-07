---
title: "Subscribers and Callbacks"
description: "Implementing robust subscribers and handling callbacks in ROS 2"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Implement subscribers that handle messages efficiently and safely
- Design callback functions that follow ROS 2 best practices
- Handle different Quality of Service (QoS) settings in subscribers
- Create robust message processing pipelines with proper error handling

## Introduction

Subscribers in ROS 2 are the counterpart to publishers and are responsible for receiving and processing messages published on topics. In humanoid robotics applications, subscribers are critical for processing sensor data, receiving commands, and reacting to system events. The effectiveness of a robot's behavior depends heavily on how well it processes incoming messages, making robust subscriber design essential for Physical AI applications.

Effective subscribers not only process messages correctly but also handle various edge cases like message loss, timing issues, and system failures. This chapter explores the implementation of subscribers that can operate reliably in dynamic robotic environments while maintaining real-time performance requirements.

## Core Concepts

Subscribers in ROS 2 are created with specific Quality of Service (QoS) settings that must be compatible with the publishers on the same topic. The callback functions associated with subscribers must be designed to handle messages efficiently without blocking other operations.

### Message Callbacks

Callback functions are executed when messages arrive. They must be designed to avoid blocking operations since ROS 2 uses a single-threaded executor by default for callbacks. Long-running operations should be handled in separate threads or through other mechanisms.

### QoS Compatibility

Subscribers must have compatible QoS settings with publishers on the same topic. While not all settings need to match exactly, they must be compatible to establish communication. Understanding these compatibility rules is important for debugging connectivity issues.

### Message Processing Pipelines

Complex robotic systems often require multiple processing steps for each message. Designing efficient processing pipelines that can handle high message rates is a key skill in robotics software development.

## Hands-on Examples

Let's implement subscribers with various processing strategies:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="basic_subscriber" label="Basic Subscriber" default>

```python
#!/usr/bin/env python3

"""
Basic subscriber example
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState


class BasicSubscriber(Node):

    def __init__(self):
        super().__init__('basic_subscriber')
        
        # Create QoS profile for subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to simple string messages
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning
        
        # Subscribe to floating point values
        self.float_subscription = self.create_subscription(
            Float32,
            'sensor_value',
            self.float_callback,
            qos_profile
        )
        
        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        self.get_logger().info('Basic subscriber initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

    def float_callback(self, msg):
        self.get_logger().info(f'Received sensor value: {msg.data:.2f}')

    def joint_state_callback(self, msg):
        if len(msg.name) > 0 and len(msg.position) > 0:
            self.get_logger().info(
                f'Received joint states for {len(msg.name)} joints, '
                f'first joint: {msg.name[0]} = {msg.position[0]:.3f}'
            )
        else:
            self.get_logger().info('Received joint state message with no data')


def main(args=None):
    rclpy.init(args=args)

    basic_subscriber = BasicSubscriber()

    try:
        rclpy.spin(basic_subscriber)
    except KeyboardInterrupt:
        basic_subscriber.get_logger().info('Subscriber stopped by user')
    finally:
        basic_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="advanced_subscriber" label="Advanced Subscriber">

```python
#!/usr/bin/env python3

"""
Advanced subscriber with buffering and state management
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import threading
import queue
from collections import deque
import time


class AdvancedSubscriber(Node):

    def __init__(self):
        super().__init__('advanced_subscriber')
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Buffer for storing recent messages
        self.message_buffer = deque(maxlen=100)  # Keep last 100 messages
        
        # Subscribe to different topics
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.status_subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            qos_profile
        )
        
        # Queue for background processing
        self.processing_queue = queue.Queue(maxsize=10)
        
        # Background thread for processing
        self.processing_thread = threading.Thread(target=self.process_messages)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        # Timer to periodically check buffers
        self.timer = self.create_timer(1.0, self.periodic_check)
        
        self.get_logger().info('Advanced subscriber with buffering initialized')

    def cmd_vel_callback(self, msg):
        # Store message in buffer with timestamp
        timestamp = self.get_clock().now()
        message_with_time = {
            'timestamp': timestamp,
            'message': msg,
            'topic': 'cmd_vel'
        }
        
        # Add to internal buffer
        self.message_buffer.append(message_with_time)
        
        # Add to processing queue for background processing
        try:
            self.processing_queue.put_nowait(message_with_time)
            self.get_logger().debug(f'Added cmd_vel message to processing queue')
        except queue.Full:
            self.get_logger().warning('Processing queue is full, dropping message')

    def status_callback(self, msg):
        # Store message in buffer with timestamp
        timestamp = self.get_clock().now()
        message_with_time = {
            'timestamp': timestamp,
            'message': msg,
            'topic': 'robot_status'
        }
        
        # Add to internal buffer
        self.message_buffer.append(message_with_time)
        
        # Log status
        self.get_logger().info(f'Status received: {msg.data}')

    def process_messages(self):
        """Background thread to process messages"""
        while rclpy.ok():
            try:
                # Get message from queue (blocking with timeout)
                message_with_time = self.processing_queue.get(timeout=0.1)
                
                # Process the message in background
                self.background_processing(message_with_time)
                
                # Mark task as done
                self.processing_queue.task_done()
            except queue.Empty:
                continue  # Check again in the next iteration

    def background_processing(self, message_with_time):
        """Process messages in background thread"""
        msg = message_with_time['message']
        topic = message_with_time['topic']
        
        # Simulate processing time
        time.sleep(0.01)  # 10ms simulated processing
        
        if topic == 'cmd_vel':
            # Process velocity command
            self.get_logger().debug(
                f'Background: Processed cmd_vel - linear.x: {msg.linear.x}, '
                f'angular.z: {msg.angular.z}'
            )
        else:
            self.get_logger().debug(f'Background: Processed {topic} message')

    def periodic_check(self):
        """Periodically check buffers and log statistics"""
        self.get_logger().info(f'Buffer size: {len(self.message_buffer)}, '
                              f'Queue size: {self.processing_queue.qsize()}')


def main(args=None):
    rclpy.init(args=args)

    advanced_subscriber = AdvancedSubscriber()

    try:
        rclpy.spin(advanced_subscriber)
    except KeyboardInterrupt:
        advanced_subscriber.get_logger().info('Advanced subscriber stopped by user')
    finally:
        advanced_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [basic_subscriber]: Basic subscriber initialized
[INFO] [1678882844.123456789] [basic_subscriber]: Received: "humanoid_robot status report: 0"
[INFO] [1678882844.123456789] [basic_subscriber]: Received sensor value: 1.23
[INFO] [1678882844.123456789] [basic_subscriber]: Received joint states for 12 joints, first joint: left_hip = 0.100
[INFO] [1678882845.123456789] [advanced_subscriber]: Advanced subscriber with buffering initialized
[INFO] [1678882846.123456789] [advanced_subscriber]: Buffer size: 1, Queue size: 0
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Message Filtering**: Create a subscriber that filters messages based on content
   - [ ] Implement a subscriber that only processes messages meeting certain criteria
   - [ ] Add filtering logic to ignore outdated messages
   - [ ] Create a statistics tracker for filtered messages
   - [ ] Test with messages that meet and don't meet criteria

2. **Threading Strategy**: Implement a subscriber that uses multiple threads for processing
   - [ ] Create a subscriber with a dedicated thread for each message type
   - [ ] Implement proper synchronization between threads
   - [ ] Measure performance differences between single and multi-threaded approaches
   - [ ] Add thread safety mechanisms to prevent data races

## Common Pitfalls and Solutions

- **Pitfall 1**: Blocking callbacks - Performing long operations in callback functions, blocking other callbacks
  - *Solution*: Move long operations to background threads or use async processing
- **Pitfall 2**: Memory issues - Storing unlimited messages in buffers without bounds
  - *Solution*: Use bounded buffers like collections.deque with a maxlen parameter
- **Pitfall 3**: Race conditions - Unsafe access to shared data in callbacks
  - *Solution*: Use locks, queues, or other thread-safe mechanisms for shared data
- **Pitfall 4**: Unhandled exceptions - Exceptions in callbacks that crash the node
  - *Solution*: Wrap callback code in try-catch blocks with proper error handling

## Summary

- Subscribers receive messages from topics using callback functions
- Callbacks should be lightweight and avoid blocking operations
- QoS settings must be compatible between publishers and subscribers
- Message buffering and background processing can improve performance
- Proper error handling is essential for robust subscriber design

## Further Reading

- [ROS 2 Subscribers Guide](https://docs.ros.org/en/humble/Tutorials/Topics/Subscribing-to-a-topic.html)
- [Threading in ROS 2](https://docs.ros.org/en/humble/How-To-Guides/Using-Executors.html)
- [Quality of Service Compatibility](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html#qos-compatibility)
- [Real-time Considerations](https://docs.ros.org/en/humble/How-To-Guides/Real-Time-Programming.html)