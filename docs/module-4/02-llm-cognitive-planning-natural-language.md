---
title: "LLM Cognitive Planning with Natural Language"
description: "Implementing cognitive planning using Large Language Models for humanoid robots"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Integrate Large Language Models (LLMs) with ROS 2 for cognitive planning
- Design natural language interfaces for high-level robot task planning
- Implement task decomposition algorithms using LLMs
- Create cognitive planning pipelines that translate natural language to robot actions

## Introduction

Large Language Models (LLMs) represent a breakthrough in artificial intelligence, enabling sophisticated natural language understanding and reasoning capabilities. In humanoid robotics, LLMs can serve as cognitive planners that interpret high-level natural language commands and decompose them into executable robot actions. This approach enables more intuitive human-robot interaction and sophisticated autonomous behavior.

This chapter explores how to integrate LLMs with ROS 2 to create cognitive planning systems for humanoid robots. We'll examine how to design interfaces that allow LLMs to interact with robot systems, how to decompose complex tasks into smaller robot actions, and how to implement reliable cognitive planning pipelines that maintain safety and correctness.

## Core Concepts

LLM-based cognitive planning involves several key components working together to translate high-level natural language commands into executable robot behaviors. The system must understand the command, reason about the world state, break down the task into executable steps, and coordinate the robot's actions.

### Task Decomposition

The core challenge in LLM-based cognitive planning is decomposing high-level tasks into a sequence of executable robot actions. This requires:

- Understanding the intent behind natural language commands
- Reasoning about the current world state and robot capabilities
- Breaking complex tasks into simpler, executable steps
- Handling dependencies and constraints between steps

### World Modeling

The cognitive planner needs a representation of the current world state and robot capabilities to determine how to execute tasks. This includes:

- Object locations and properties
- Robot state and capabilities
- Environmental constraints
- Safety and operational limits

### Safety and Validation

LLM responses must be validated to ensure safe and appropriate robot behavior. This includes:

- Checking for safety violations in proposed actions
- Verifying that the robot can actually perform requested actions
- Handling ambiguous or impossible requests appropriately

## Hands-on Examples

Let's implement an LLM-based cognitive planning system:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="llm_planner" label="LLM Cognitive Planner" default>

```python
#!/usr/bin/env python3

"""
LLM-based cognitive planning node for humanoid robots
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
import json
import openai
import time
from typing import List, Dict, Any, Optional
import threading
import queue


class LLMCognitivePlanner(Node):

    def __init__(self):
        super().__init__('llm_cognitive_planner')
        
        # Publishers
        self.plan_publisher = self.create_publisher(
            String,
            'cognitive_plan',
            QoSProfile(depth=10)
        )
        
        self.status_publisher = self.create_publisher(
            String,
            'cognitive_status',
            QoSProfile(depth=10)
        )
        
        # Subscribers
        self.command_subscription = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            QoSProfile(depth=10)
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            QoSProfile(depth=10)
        )
        
        # Internal state
        self.robot_pose = Pose()
        self.joint_positions = {}
        self.current_task_queue = queue.Queue()
        self.is_executing_plan = False
        
        # Initialize OpenAI API key (in real system, this would be from parameters)
        # openai.api_key = os.getenv("OPENAI_API_KEY")
        
        # Robot capabilities definition
        self.robot_capabilities = [
            "move forward/backward", 
            "turn left/right",
            "navigate to location",
            "pick up object", 
            "place object",
            "wave", 
            "greet",
            "speak text",
            "detect objects",
            "grasp object"
        ]
        
        self.get_logger().info('LLM Cognitive Planner initialized')

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry"""
        self.robot_pose = msg.pose.pose

    def command_callback(self, msg: String):
        """Process natural language command"""
        command = msg.data
        self.get_logger().info(f'Received natural language command: "{command}"')
        
        # Publish system status
        status_msg = String()
        status_msg.data = f'Processing: {command}'
        self.status_publisher.publish(status_msg)
        
        # Plan the task using LLM in a separate thread to avoid blocking
        planning_thread = threading.Thread(
            target=self.plan_task_with_llm, 
            args=(command,)
        )
        planning_thread.start()

    def plan_task_with_llm(self, command: str):
        """Generate a plan using LLM for the given command"""
        try:
            # Define the system prompt for the LLM
            system_prompt = f"""
            You are a cognitive planner for a humanoid robot. Your role is to decompose high-level natural language commands 
            into executable tasks for a humanoid robot. The robot has the following capabilities:
            {', '.join(self.robot_capabilities)}.
            
            Current robot state: At position {self.robot_pose.position.x:.2f}, {self.robot_pose.position.y:.2f}.
            
            For each command, respond with a JSON object that contains a 'plan' array of executable steps. Each step should have:
            - 'action': The specific action to perform
            - 'target': The target of the action (if applicable)
            - 'parameters': Any parameters needed for the action
            
            Example response format:
            {{
                "plan": [
                    {{"action": "navigate_to", "target": "kitchen", "parameters": {{"x": 1.0, "y": 2.0}}}},
                    {{"action": "detect_object", "target": "coffee mug", "parameters": {{}}}},
                    {{"action": "grasp_object", "target": "coffee mug", "parameters": {{"approach": "top"}}}}
                ],
                "description": "A brief description of the plan"
            }}
            
            Respond only with valid JSON, nothing else.
            """
            
            # In a real implementation, this would call the LLM API
            # For this example, we'll simulate the response
            simulated_plan = self.simulate_llm_response(command)
            
            if simulated_plan:
                # Publish the generated plan
                plan_msg = String()
                plan_msg.data = json.dumps(simulated_plan)
                self.plan_publisher.publish(plan_msg)
                
                self.get_logger().info(f'Plan generated for command: {command}')
                self.get_logger().debug(f'Plan: {json.dumps(simulated_plan, indent=2)}')
                
                # Execute the plan
                self.execute_plan(simulated_plan['plan'])
        
        except Exception as e:
            self.get_logger().error(f'Error generating plan with LLM: {e}')
            # Publish error status
            status_msg = String()
            status_msg.data = f'Planning error: {str(e)}'
            self.status_publisher.publish(status_msg)

    def simulate_llm_response(self, command: str) -> Optional[Dict[str, Any]]:
        """Simulate LLM response for the command"""
        # In a real system, this would call the actual LLM
        # For simulation, we'll create appropriate responses based on command content
        
        command_lower = command.lower()
        
        if "bring me coffee" in command_lower:
            return {
                "plan": [
                    {"action": "navigate_to", "target": "kitchen", "parameters": {"x": 3.0, "y": 2.5}},
                    {"action": "detect_object", "target": "coffee mug", "parameters": {}},
                    {"action": "grasp_object", "target": "coffee mug", "parameters": {"approach": "top"}},
                    {"action": "navigate_to", "target": "living room", "parameters": {"x": 0.0, "y": 0.0}},
                    {"action": "place_object", "target": "table", "parameters": {"height": 0.8}}
                ],
                "description": "Fetch coffee from kitchen and bring to user"
            }
        elif "wave hello" in command_lower or "greet" in command_lower:
            return {
                "plan": [
                    {"action": "wave", "target": "", "parameters": {}},
                    {"action": "speak_text", "target": "Hello! It's nice to meet you.", "parameters": {"volume": 0.8}}
                ],
                "description": "Wave and greet the user"
            }
        elif "go to" in command_lower or "navigate to" in command_lower:
            # Extract location from command
            if "kitchen" in command_lower:
                location = "kitchen"
                x, y = 3.0, 2.5
            elif "bedroom" in command_lower:
                location = "bedroom"
                x, y = -2.0, 1.0
            elif "living room" in command_lower:
                location = "living room"
                x, y = 0.0, 0.0
            else:
                location = "unknown"
                x, y = 1.0, 1.0  # default location
            
            return {
                "plan": [
                    {"action": "navigate_to", "target": location, "parameters": {"x": x, "y": y}}
                ],
                "description": f"Navigate to {location}"
            }
        else:
            # Default response for unrecognized commands
            return {
                "plan": [
                    {"action": "speak_text", "target": f"I'm not sure how to '{command}'. Can you rephrase?", "parameters": {"volume": 0.8}}
                ],
                "description": "Ask for clarification"
            }

    def execute_plan(self, plan: List[Dict[str, Any]]):
        """Execute a plan consisting of action steps"""
        if self.is_executing_plan:
            self.get_logger().warn('Already executing a plan, ignoring new plan')
            return
        
        self.is_executing_plan = True
        self.get_logger().info(f'Starting execution of plan with {len(plan)} steps')
        
        for i, step in enumerate(plan):
            self.get_logger().info(f'Executing step {i+1}/{len(plan)}: {step["action"]}')
            
            # Status update
            status_msg = String()
            status_msg.data = f'Executing: {step["action"]} ({i+1}/{len(plan)})'
            self.status_publisher.publish(status_msg)
            
            success = self.execute_single_step(step)
            
            if not success:
                self.get_logger().error(f'Step {i+1} failed: {step["action"]}. Aborting plan.')
                break
            
            # Small delay between steps to allow for proper execution
            time.sleep(0.5)
        
        self.is_executing_plan = False
        self.get_logger().info('Plan execution completed')
        
        # Publish completion status
        status_msg = String()
        status_msg.data = 'Plan completed'
        self.status_publisher.publish(status_msg)

    def execute_single_step(self, step: Dict[str, Any]) -> bool:
        """Execute a single step in the plan"""
        action = step["action"]
        target = step["target"]
        params = step["parameters"]
        
        self.get_logger().debug(f'Executing action: {action}, target: {target}, params: {params}')
        
        # Execute the action based on its type
        if action == "navigate_to":
            return self.execute_navigation(target, params)
        elif action == "detect_object":
            return self.execute_object_detection(target, params)
        elif action == "grasp_object":
            return self.execute_grasp(target, params)
        elif action == "place_object":
            return self.execute_place(target, params)
        elif action == "wave":
            return self.execute_wave(params)
        elif action == "speak_text":
            return self.execute_speak(target, params)
        else:
            self.get_logger().warn(f'Unknown action type: {action}')
            return False

    def execute_navigation(self, target: str, params: Dict[str, Any]) -> bool:
        """Execute navigation action"""
        # In a real implementation, this would call navigation action
        # For this example, we'll simulate the action
        x = params.get("x", 0.0)
        y = params.get("y", 0.0)
        
        self.get_logger().info(f'Navigating to {target} at ({x}, {y})')
        
        # Simulate navigation time
        time.sleep(2.0)
        
        # Update robot pose to new position
        self.robot_pose.position.x = x
        self.robot_pose.position.y = y
        
        return True

    def execute_object_detection(self, target: str, params: Dict[str, Any]) -> bool:
        """Execute object detection action"""
        self.get_logger().info(f'Detecting object: {target}')
        
        # Simulate detection time
        time.sleep(1.0)
        
        # For simulation, assume object is found
        self.get_logger().info(f'Object {target} detected')
        
        return True

    def execute_grasp(self, target: str, params: Dict[str, Any]) -> bool:
        """Execute grasping action"""
        approach = params.get("approach", "top")
        self.get_logger().info(f'Grasping {target} using {approach} approach')
        
        # Simulate grasping time
        time.sleep(1.5)
        
        self.get_logger().info(f'Object {target} grasped successfully')
        
        return True

    def execute_place(self, target: str, params: Dict[str, Any]) -> bool:
        """Execute placing action"""
        height = params.get("height", 1.0)
        self.get_logger().info(f'Placing object on {target} at height {height}')
        
        # Simulate placement time
        time.sleep(1.0)
        
        self.get_logger().info(f'Object placed on {target}')
        
        return True

    def execute_wave(self, params: Dict[str, Any]) -> bool:
        """Execute waving action"""
        self.get_logger().info('Waving')
        
        # Simulate waving time
        time.sleep(1.0)
        
        self.get_logger().info('Wave motion completed')
        
        return True

    def execute_speak(self, text: str, params: Dict[str, Any]) -> bool:
        """Execute speech action"""
        volume = params.get("volume", 0.5)
        self.get_logger().info(f'Speaking: "{text}" at volume {volume}')
        
        # Simulate speaking time
        time.sleep(len(text) * 0.1)  # Roughly 0.1 seconds per word
        
        self.get_logger().info('Speech completed')
        
        return True


def main(args=None):
    rclpy.init(args=args)

    cognitive_planner = LLMCognitivePlanner()

    try:
        rclpy.spin(cognitive_planner)
    except KeyboardInterrupt:
        cognitive_planner.get_logger().info('Cognitive planner stopped by user')
    finally:
        cognitive_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="task_decomposition" label="Task Decomposition">

```python
#!/usr/bin/env python3

"""
Task decomposition module for LLM cognitive planning
"""

import json
import openai
from typing import List, Dict, Any, Tuple
import re


class TaskDecomposer:
    """
    Component for decomposing high-level tasks into executable robot actions
    """
    
    def __init__(self, robot_capabilities: List[str], environment_map: Dict[str, Any] = None):
        self.robot_capabilities = robot_capabilities
        self.environment_map = environment_map or {}
    
    def decompose_task(self, natural_language_command: str) -> Tuple[List[Dict[str, Any]], str]:
        """
        Decompose a natural language command into a sequence of executable actions
        
        Returns:
            - List of action dictionaries
            - Natural language description of the plan
        """
        # In a real implementation, this would call the LLM
        # For this example, we'll use rule-based decomposition
        return self._rule_based_decomposition(natural_language_command)
    
    def _rule_based_decomposition(self, command: str) -> Tuple[List[Dict[str, Any]], str]:
        """
        Rule-based task decomposition (simulated LLM response)
        """
        command_lower = command.lower()
        
        # Define decomposition rules
        if "fetch" in command_lower or "get" in command_lower or "bring" in command_lower:
            # Extract object to fetch
            obj_match = re.search(r'(?:fetch|get|bring) (?:me )?([^.,!?]+)', command_lower)
            obj = obj_match.group(1).strip() if obj_match else "item"
            
            # Extract destination
            dest_match = re.search(r'to (?:the )?([^.,!?]+)', command_lower)
            destination = dest_match.group(1).strip() if dest_match else "here"
            
            plan = [
                {
                    "action": "find_object", 
                    "target": obj,
                    "parameters": {"search_area": self._get_search_area_for_object(obj)},
                    "description": f"Locate the {obj}"
                },
                {
                    "action": "navigate_to", 
                    "target": obj,
                    "parameters": {"approach_distance": 0.5},
                    "description": f"Move to the {obj}"
                },
                {
                    "action": "grasp_object", 
                    "target": obj,
                    "parameters": {"grasp_type": "pinch"},
                    "description": f"Pick up the {obj}"
                },
                {
                    "action": "navigate_to", 
                    "target": destination,
                    "parameters": {"approach_distance": 1.0},
                    "description": f"Go to {destination}"
                },
                {
                    "action": "place_object", 
                    "target": destination,
                    "parameters": {"placement_height": 0.8},
                    "description": f"Put the {obj} down at {destination}"
                }
            ]
            
            description = f"Fetch {obj} and bring it to {destination}"
            
        elif "go to" in command_lower or "move to" in command_lower or "navigate to" in command_lower:
            # Extract destination
            dest_match = re.search(r'(?:go to|move to|navigate to) (?:the )?([^.,!?]+)', command_lower)
            destination = dest_match.group(1).strip() if dest_match else "location"
            
            # Get coordinates from environment map
            x, y = self._get_coordinates_for_location(destination)
            
            plan = [
                {
                    "action": "navigate_to", 
                    "target": destination,
                    "parameters": {"x": x, "y": y},
                    "description": f"Navigate to {destination} at ({x}, {y})"
                }
            ]
            
            description = f"Go to {destination}"
            
        elif "clean" in command_lower:
            # Extract room to clean
            room_match = re.search(r'clean (?:the )?([^.,!?]+)', command_lower)
            room = room_match.group(1).strip() if room_match else "room"
            
            plan = [
                {
                    "action": "navigate_to", 
                    "target": f"{room}_entry",
                    "parameters": {"approach_distance": 0.5},
                    "description": f"Enter the {room}"
                },
                {
                    "action": "perform_cleaning_pattern", 
                    "target": room,
                    "parameters": {"pattern": "spiral", "coverage": 0.95},
                    "description": f"Clean the {room} in a spiral pattern"
                },
                {
                    "action": "navigate_to", 
                    "target": f"{room}_exit",
                    "parameters": {"approach_distance": 0.5},
                    "description": f"Exit the {room}"
                }
            ]
            
            description = f"Clean the {room}"
            
        else:
            # Default response for unrecognized commands
            plan = [
                {
                    "action": "speak_text", 
                    "target": f"I'm not sure how to '{command}'. Can you rephrase?",
                    "parameters": {"volume": 0.8},
                    "description": f"Ask for clarification about '{command}'"
                }
            ]
            
            description = f"Ask for clarification about '{command}'"
        
        return plan, description
    
    def _get_search_area_for_object(self, obj: str) -> str:
        """Get likely search area for an object"""
        # Simple mapping - in real system would use object recognition and spatial memory
        object_locations = {
            "coffee": "kitchen",
            "book": "living room",
            "water": "kitchen",
            "keys": "entryway",
            "phone": "living room"
        }
        return object_locations.get(obj, "general_area")
    
    def _get_coordinates_for_location(self, location: str) -> Tuple[float, float]:
        """Get coordinates for a named location"""
        # Simple mapping - in real system would use semantic map
        location_coords = {
            "kitchen": (3.0, 2.5),
            "bedroom": (-2.0, 1.0),
            "living room": (0.0, 0.0),
            "bathroom": (-1.5, -2.0),
            "office": (2.0, -1.5)
        }
        return location_coords.get(location, (1.0, 1.0))  # default coordinates


# Example usage
if __name__ == "__main__":
    # Define robot capabilities
    capabilities = [
        "navigate_to",
        "grasp_object", 
        "place_object",
        "find_object",
        "speak_text",
        "perform_cleaning_pattern"
    ]
    
    # Initialize decomposer
    decomposer = TaskDecomposer(capabilities)
    
    # Test commands
    test_commands = [
        "Bring me a coffee from the kitchen",
        "Go to the living room",
        "Clean the bedroom"
    ]
    
    for cmd in test_commands:
        print(f"\nCommand: {cmd}")
        plan, description = decomposer.decompose_task(cmd)
        print(f"Plan description: {description}")
        print("Steps:")
        for i, step in enumerate(plan, 1):
            print(f"  {i}. {step['description']}")
        print(json.dumps({"plan": plan}, indent=2))
```

</TabItem>
</Tabs>

Expected Output:
```
Command: Bring me a coffee from the kitchen
Plan description: Fetch coffee and bring it to here
Steps:
  1. Locate the coffee
  2. Move to the coffee
  3. Pick up the coffee
  4. Go to here
  5. Put the coffee down at here
  
{
  "plan": [
    {
      "action": "find_object",
      "target": "coffee",
      "parameters": {"search_area": "kitchen"},
      "description": "Locate the coffee"
    },
    {
      "action": "navigate_to", 
      "target": "coffee",
      "parameters": {"approach_distance": 0.5},
      "description": "Move to the coffee"
    },
    {
      "action": "grasp_object", 
      "target": "coffee",
      "parameters": {"grasp_type": "pinch"},
      "description": "Pick up the coffee"
    },
    {
      "action": "navigate_to", 
      "target": "here",
      "parameters": {"approach_distance": 1.0},
      "description": "Go to here"
    },
    {
      "action": "place_object", 
      "target": "here",
      "parameters": {"placement_height": 0.8},
      "description": "Put the coffee down at here"
    }
  ]
}

[INFO] [1678882844.123456789] [llm_cognitive_planner]: LLM Cognitive Planner initialized
[INFO] [1678882845.123456789] [llm_cognitive_planner]: Received natural language command: "Bring me a coffee from the kitchen"
[INFO] [1678882845.223456789] [llm_cognitive_planner]: Plan generated for command: Bring me a coffee from the kitchen
[INFO] [1678882845.223456789] [llm_cognitive_planner]: Starting execution of plan with 5 steps
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Plan Validation**: Implement plan validation before execution
   - [ ] Create a validator that checks if all actions are supported by the robot
   - [ ] Implement safety checks to prevent dangerous action sequences
   - [ ] Add simulation capabilities to test plans before execution
   - [ ] Create a confirmation step for high-risk actions

2. **Context Awareness**: Enhance the planner with environmental awareness
   - [ ] Integrate with ROS 2 navigation stack for real location data
   - [ ] Add object detection integration to update world model
   - [ ] Implement dynamic replanning when environment changes
   - [ ] Create a semantic map for named locations

## Common Pitfalls and Solutions

- **Pitfall 1**: LLM hallucinations - LLMs generating invalid or unsafe commands
  - *Solution*: Implement strict output parsing and validation layers
- **Pitfall 2**: Context window limitations - LLMs forgetting robot capabilities
  - *Solution*: Design compact, comprehensive system prompts and use function calling
- **Pitfall 3**: Inconsistent planning - Same command generating different plans
  - *Solution*: Use structured prompts with clear examples and constraints
- **Pitfall 4**: Safety violations - Plans that could harm the robot or environment
  - *Solution*: Implement safety validation layers and constraint checking

## Summary

- LLMs enable natural language interfaces for robot cognitive planning
- Task decomposition translates high-level commands to executable actions
- Safety and validation layers are essential for reliable operation
- Context awareness improves plan relevance and feasibility
- Proper system design handles LLM limitations and safety requirements

## Further Reading

- [OpenAI Function Calling](https://platform.openai.com/docs/guides/function-calling)
- [ROS 2 Navigation Stack](https://navigation.ros.org/)
- [Language-Enabled Robotics](https://arxiv.org/abs/2206.07889)
- [Large Language Models for Robotics](https://arxiv.org/abs/2309.13874)