---
title: "Voice-to-Action with OpenAI Whisper"
description: "Implementing speech recognition for humanoid robot control"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Integrate OpenAI Whisper for speech recognition in ROS 2
- Design voice command vocabularies for humanoid robot control
- Implement robust voice command processing pipelines
- Handle speech recognition errors and uncertainties

## Introduction

Voice-to-action systems bridge the gap between natural human language and robot actions, enabling intuitive human-robot interaction. In Physical AI applications with humanoid robots, voice interfaces provide a natural way for humans to communicate tasks and commands to robots. OpenAI's Whisper model offers state-of-the-art speech recognition capabilities, making it an excellent choice for implementing voice-controlled robotic systems.

This chapter explores how to integrate Whisper with ROS 2 to create robust voice-to-action systems for humanoid robots. We'll cover the integration process, command vocabulary design, and error handling strategies essential for reliable voice-controlled robot operation.

## Core Concepts

Implementing voice-to-action systems involves several key components that must work together to provide reliable and responsive interaction. The system must capture audio, process it through a speech recognition model, interpret the recognized text, and convert it to appropriate robot actions.

### Speech Recognition Pipeline

The pipeline typically involves:
1. Audio capture from microphones
2. Preprocessing of audio signals
3. Speech-to-text conversion using Whisper
4. Natural language processing of the recognized text
5. Mapping to specific robot commands
6. Execution of robot actions

### Command Vocabulary Design

Designing an effective command vocabulary is crucial for system usability. Commands should be:
- Unambiguous and distinct from each other
- Easy to pronounce and remember
- Appropriate for the robot's capabilities
- Robust to variations in pronunciation

### Error Handling and Confidence

Speech recognition systems must handle uncertainty and errors gracefully. This includes recognizing when the system is not confident about a recognition, handling unrecognized commands, and providing feedback to the user.

## Hands-on Examples

Let's implement a voice-to-action system with OpenAI Whisper:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="whisper_node" label="Whisper Integration Node" default>

```python
#!/usr/bin/env python3

"""
Voice command recognition node using OpenAI Whisper
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
import pyaudio
import wave
import threading
import time
import queue
import openai
import os
import json

# Configuration constants
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000  # Whisper works well with 16kHz
RECORD_SECONDS = 3
WAKE_WORD = "hey robot"

# Command vocabulary mapping recognized text to robot actions
COMMAND_VOCABULARY = {
    "move forward": "forward",
    "go forward": "forward",
    "move back": "backward", 
    "go back": "backward",
    "move backward": "backward",
    "turn left": "left",
    "rotate left": "left",
    "turn right": "right",
    "rotate right": "right",
    "stop": "stop",
    "halt": "stop",
    "look around": "look_around",
    "raise your hand": "raise_hand",
    "lower your hand": "lower_hand",
    "wave": "wave",
    "dance": "dance"
}


class VoiceCommandNode(Node):

    def __init__(self):
        super().__init__('voice_command_node')
        
        # Initialize audio interface
        self.audio = pyaudio.PyAudio()
        self.is_listening = False
        self.audio_queue = queue.Queue()
        
        # Publishers
        self.voice_text_publisher = self.create_publisher(
            String,
            'voice_text',
            QoSProfile(depth=10)
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(depth=10)
        )
        
        # Timer for audio processing
        self.audio_timer = self.create_timer(0.1, self.process_audio)
        
        # Start audio capture thread
        self.audio_thread = threading.Thread(target=self.capture_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        # Initialize Whisper API key (in a real system, this would come from parameters)
        # openai.api_key = os.getenv("OPENAI_API_KEY")
        
        self.get_logger().info('Voice command node initialized')

    def capture_audio(self):
        """Capture audio from microphone and add to queue"""
        stream = self.audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK
        )
        
        self.get_logger().info('Audio capture started')
        
        while rclpy.ok():
            try:
                # Read audio data
                data = stream.read(CHUNK, exception_on_overflow=False)
                self.audio_queue.put(data)
            except Exception as e:
                self.get_logger().error(f'Audio capture error: {e}')
                break
        
        stream.stop_stream()
        stream.close()

    def process_audio(self):
        """Process audio data and recognize speech"""
        # In a real implementation, we would collect audio chunks,
        # send them to Whisper, and process the recognized text
        # For this example, we'll simulate the process
        
        if not self.audio_queue.empty():
            # Collect audio data from queue
            audio_frames = []
            while not self.audio_queue.empty():
                audio_frames.append(self.audio_queue.get())
            
            if len(audio_frames) > 0:
                # In a real system, we would send audio_frames to Whisper
                # For simulation, we'll generate recognized text based on volume
                # (in practice, this would come from Whisper API)
                
                # Simulate recognition of a command
                if np.random.random() > 0.95:  # Occasionally recognize something
                    possible_commands = list(COMMAND_VOCABULARY.keys())
                    recognized_text = np.random.choice(possible_commands)
                    
                    # Publish recognized text
                    text_msg = String()
                    text_msg.data = recognized_text
                    self.voice_text_publisher.publish(text_msg)
                    
                    self.get_logger().info(f'Recognized: "{recognized_text}"')
                    
                    # Process the recognized command
                    self.process_recognized_command(recognized_text)

    def process_recognized_command(self, text):
        """Process recognized text and generate robot commands"""
        # Convert to lowercase and clean
        clean_text = text.lower().strip()
        
        # Check if command exists in vocabulary
        if clean_text in COMMAND_VOCABULARY:
            command = COMMAND_VOCABULARY[clean_text]
            self.execute_robot_command(command)
        else:
            # Check for partial matches or handle unrecognized command
            self.get_logger().info(f'Unrecognized command: "{clean_text}"')
            # Could implement fuzzy matching here

    def execute_robot_command(self, command):
        """Execute the corresponding robot action"""
        self.get_logger().info(f'Executing command: {command}')
        
        twist_msg = Twist()
        
        if command == "forward":
            twist_msg.linear.x = 0.5  # Move forward at 0.5 m/s
        elif command == "backward":
            twist_msg.linear.x = -0.5  # Move backward at 0.5 m/s
        elif command == "left":
            twist_msg.angular.z = 0.5  # Turn counter-clockwise
        elif command == "right":
            twist_msg.angular.z = -0.5  # Turn clockwise  
        elif command == "stop":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        # Additional commands would be implemented here
        
        # Publish the command
        self.cmd_vel_publisher.publish(twist_msg)

    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    voice_command_node = VoiceCommandNode()

    try:
        rclpy.spin(voice_command_node)
    except KeyboardInterrupt:
        voice_command_node.get_logger().info('Voice command node stopped by user')
    finally:
        voice_command_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="command_interpreter" label="Command Interpreter">

```python
#!/usr/bin/env python3

"""
Command interpreter for voice-to-action system
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
import re


class VoiceCommandInterpreter(Node):

    def __init__(self):
        super().__init__('voice_command_interpreter')
        
        # Subscribe to recognized text
        self.text_subscription = self.create_subscription(
            String,
            'voice_text',
            self.text_callback,
            QoSProfile(depth=10)
        )
        
        # Publishers for different types of commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(depth=10)
        )
        
        # Store command patterns and their handlers
        self.command_patterns = [
            # Movement commands
            (r"move\s+(?P<direction>forward|backward|back|up|down|left|right)\s*(?P<distance>\d+\.?\d*)?\s*(m|meter|cm)?", self.handle_directional_move),
            (r"go\s+(?P<direction>forward|backward|back|up|down|left|right)\s*(?P<duration>\d+\.?\d*)?\s*(s|second|seconds)?", self.handle_directional_move),
            (r"turn\s+(?P<direction>left|right)\s*(?P<angle>\d+\.?\d*)?\s*(deg|degree|degrees)?", self.handle_rotation),
            
            # Navigation commands
            (r"go\s+to\s+(?P<location>\w+)", self.handle_navigation),
            (r"navigate\s+to\s+(?P<location>\w+)", self.handle_navigation),
            
            # Action commands  
            (r"(?P<action>wave|dance|stop|halt|freeze|wait)", self.handle_action),
            
            # Complex commands
            (r"move\s+(?P<x>-?\d+\.?\d*)\s*m\s+and\s+(?P<y>-?\d+\.?\d*)\s*m", self.handle_complex_move),
        ]
        
        self.get_logger().info('Voice command interpreter initialized')

    def text_callback(self, msg):
        """Process incoming text commands"""
        text = msg.data.lower().strip()
        self.get_logger().info(f'Received text: "{text}"')
        
        # Try to match command patterns
        matched = False
        for pattern, handler in self.command_patterns:
            match = re.search(pattern, text)
            if match:
                handler(match.groupdict())
                matched = True
                break
        
        if not matched:
            self.get_logger().info(f'No matching command for: "{text}"')

    def handle_directional_move(self, params):
        """Handle directional movement commands"""
        direction = params.get('direction', '')
        distance = float(params.get('distance', 1.0) or 1.0)  # Default to 1 meter
        
        twist_msg = Twist()
        
        if direction in ['forward', 'up']:
            twist_msg.linear.x = 0.5  # m/s
        elif direction in ['backward', 'back', 'down']:
            twist_msg.linear.x = -0.5  # m/s
        elif direction == 'left':
            twist_msg.angular.z = 0.5  # rad/s
        elif direction == 'right':
            twist_msg.angular.z = -0.5  # rad/s
            
        # If distance is specified, calculate how long to move
        duration = Duration()
        if distance > 0:
            # Assume 0.5 m/s speed for linear movement
            duration.sec = int(distance / 0.5)
            duration.nanosec = int((distance / 0.5 - duration.sec) * 1e9)
        
        self.get_logger().info(f'Executing directional move: {direction} for {distance}m')
        
        # Execute the movement using action or timed publishing
        self.execute_timed_command(twist_msg, duration)

    def handle_rotation(self, params):
        """Handle rotation commands"""
        direction = params.get('direction', 'left')
        angle = float(params.get('angle', 90.0) or 90.0)  # Default to 90 degrees
        
        # Convert angle to angular displacement
        angular_velocity = 0.5  # rad/s
        duration = Duration()
        duration.sec = int(angle / 180.0 * 3.14159 / angular_velocity)
        duration.nanosec = int((angle / 180.0 * 3.14159 / angular_velocity - duration.sec) * 1e9)
        
        twist_msg = Twist()
        if direction == 'left':
            twist_msg.angular.z = angular_velocity
        else:  # right
            twist_msg.angular.z = -angular_velocity
        
        self.get_logger().info(f'Executing rotation: {direction} by {angle} degrees')
        
        self.execute_timed_command(twist_msg, duration)

    def handle_navigation(self, params):
        """Handle navigation to specific locations"""
        location = params.get('location', '')
        self.get_logger().info(f'Navigating to location: {location}')
        
        # In a real system, this would call navigation action
        # For now, we'll just log the intent
        # self.call_navigation_action(location)

    def handle_action(self, params):
        """Handle simple action commands"""
        action = params.get('action', '')
        self.get_logger().info(f'Executing action: {action}')
        
        # Implement action-specific logic
        if action == 'wave':
            self.execute_wave_action()
        elif action == 'dance':
            self.execute_dance_action()
        elif action in ['stop', 'halt', 'freeze']:
            self.stop_robot()
        elif action == 'wait':
            self.wait_action()

    def handle_complex_move(self, params):
        """Handle complex movement commands with x,y coordinates"""
        x = float(params.get('x', 0.0))
        y = float(params.get('y', 0.0))
        self.get_logger().info(f'Executing complex move to: ({x}, {y})')
        
        # In a real system, this would calculate path and execute movement
        # For now, we'll just log the intent
        # self.calculate_path_and_move(x, y)

    def execute_timed_command(self, twist_msg, duration):
        """Execute a command for a specified duration"""
        # This is a simplified implementation
        # In a real system, you'd use a timed publisher or action
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Stop after duration (in a real implementation, use proper timing)
        time.sleep(duration.sec + duration.nanosec / 1e9)
        
        # Stop the robot
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

    def execute_wave_action(self):
        """Execute waving gesture (would control arm joints in real system)"""
        self.get_logger().info('Executing wave action')
        # In a real system, this would publish to joint controllers

    def execute_dance_action(self):
        """Execute dancing sequence"""
        self.get_logger().info('Executing dance action')
        # In a real system, this would execute a pre-programmed sequence

    def stop_robot(self):
        """Stop all robot movement"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info('Robot stopped')

    def wait_action(self):
        """Wait/pause robot actions"""
        self.get_logger().info('Robot waiting')


def main(args=None):
    rclpy.init(args=args)

    interpreter = VoiceCommandInterpreter()

    try:
        rclpy.spin(interpreter)
    except KeyboardInterrupt:
        interpreter.get_logger().info('Command interpreter stopped by user')
    finally:
        interpreter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [voice_command_node]: Voice command node initialized
[INFO] [1678882844.123456789] [voice_command_interpreter]: Voice command interpreter initialized
[INFO] [1678882845.123456789] [voice_command_node]: Recognized: "move forward"
[INFO] [1678882845.123456789] [voice_command_node]: Executing command: forward
[INFO] [1678882845.123456789] [voice_command_interpreter]: Received text: "move forward"
[INFO] [1678882845.123456789] [voice_command_interpreter]: Executing directional move: forward for 1.0m
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Command Vocabulary Extension**: Add more commands to the vocabulary
   - [ ] Implement commands for arm gestures (raise arms, point, etc.)
   - [ ] Add navigation commands to specific locations
   - [ ] Include safety commands (emergency stop, pause, etc.)
   - [ ] Test recognition accuracy with new commands

2. **Robustness Improvements**: Enhance the system's reliability
   - [ ] Add confidence thresholding for Whisper recognition
   - [ ] Implement confirmation prompts for critical commands
   - [ ] Add timeout for command execution
   - [ ] Create a feedback mechanism to confirm command execution

## Common Pitfalls and Solutions

- **Pitfall 1**: Network dependencies - Whisper API requires internet and can fail
  - *Solution*: Implement offline fallback or local speech recognition
- **Pitfall 2**: Recognition errors - Whisper may misinterpret commands
  - *Solution*: Add confirmation steps and confidence thresholds
- **Pitfall 3**: Audio quality issues - Background noise affects recognition
  - *Solution*: Implement audio preprocessing and noise reduction
- **Pitfall 4**: Command ambiguity - Similar-sounding commands causing confusion
  - *Solution*: Design clear command vocabularies with distinct terms

## Summary

- Voice-to-action systems bridge natural language and robot control
- OpenAI Whisper provides high-quality speech recognition capabilities
- Command interpretation requires careful vocabulary design
- Robust error handling is essential for reliable operation
- Voice interfaces enhance intuitive human-robot interaction

## Further Reading

- [OpenAI Whisper Documentation](https://platform.openai.com/docs/guides/speech-to-text)
- [ROS 2 Audio Processing](https://index.ros.org/r/audio_common/)
- [Speech Recognition in Robotics](https://ieeexplore.ieee.org/document/7472048)
- [Human-Robot Interaction Patterns](https://humanrobotinteraction.org/)