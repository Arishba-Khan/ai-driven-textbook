---
title: "Physics Simulation: Gravity and Collisions"
description: "Understanding and configuring physics in Gazebo simulation"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Configure physics engines and parameters in Gazebo
- Set up realistic gravity and collision properties
- Fine-tune physics parameters for humanoid robots
- Debug physics-related simulation issues

## Introduction

Physics simulation forms the foundation of realistic robotic simulation, enabling robots to interact with their environment in ways that closely mirror real-world behavior. In the context of Physical AI & Humanoid Robotics, accurate physics simulation is especially critical due to the complex multi-body dynamics involved in humanoid locomotion and interaction. Properly configured physics parameters ensure that behaviors learned in simulation can transfer effectively to real-world robots.

Gazebo supports multiple physics engines (ODE, Bullet, DART) and provides extensive control over physical parameters such as gravity, friction, damping, and collision behavior. For humanoid robots with many degrees of freedom, careful tuning of these parameters is essential to achieve stable, realistic movement patterns that will translate well to physical hardware.

## Core Concepts

Physics simulation in Gazebo involves modeling the fundamental forces and interactions that govern how objects move and interact. The accuracy of this simulation directly impacts the validity of results obtained from simulation-based development and testing.

### Physics Engines

Gazebo supports three primary physics engines:
- **ODE (Open Dynamics Engine)**: Default engine, good for general applications
- **Bullet**: Known for robust collision detection and realistic physics
- **DART**: Advanced engine with articulated body handling, good for complex robots

### Gravity and World Physics

Gravity is a fundamental force in physics simulation, and its setting affects all objects in the simulation. By default, Gazebo simulates Earth gravity (9.8 m/s²), but this can be adjusted for different environments or testing scenarios.

### Collision Detection

Collision detection algorithms determine when and how objects in the simulation interact. Proper collision geometry is critical for realistic interactions while maintaining simulation performance. For humanoid robots, this is particularly important for tasks like walking, grasping, and navigating.

## Hands-on Examples

Let's configure physics parameters for realistic humanoid simulation:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="physics_world" label="Physics-Enabled World File" default>

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="humanoid_physics_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      
      <!-- ODE-specific parameters -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Humanoid robot with custom physics properties -->
    <model name="simple_humanoid">
      <pose>0 0 1.0 0 0 0</pose>
      
      <!-- Torso with realistic mass and inertia -->
      <link name="torso">
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <mass>15.0</mass>
          <inertia>
            <ixx>0.2</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.3</iyy>
            <iyz>0.0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        
        <!-- Visual and collision geometry -->
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.2 0.5</size>
            </box>
          </geometry>
        </visual>
        
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.2 0.5</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.5</mu>
                <mu2>0.5</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
              </ode>
            </friction>
            <bounce>
              <restitution_coefficient>0.1</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
            <contact>
              <ode>
                <soft_cfm>0.0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1000000000000.0</kp>
                <kd>1.0</kd>
                <max_vel>100.0</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
      </link>
      
      <!-- Hip link with specific physics properties -->
      <link name="hip">
        <pose>-0.1 0 -0.25 0 0 0</pose>
        <inertial>
          <mass>3.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.005</izz>
          </inertia>
        </inertial>
        
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.15</length>
            </cylinder>
          </geometry>
        </visual>
        
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.15</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <soft_cfm>0.0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1000000000000.0</kp>
                <kd>1.0</kd>
              </ode>
            </contact>
          </surface>
        </collision>
      </link>
      
      <!-- Joint with actuation parameters -->
      <joint name="torso_to_hip" type="revolute">
        <parent>torso</parent>
        <child>hip</child>
        <pose>-0.1 0 -0.25 0 0 0</pose>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-1.57</lower>
            <upper>1.57</upper>
            <effort>50.0</effort>
            <velocity>2.0</velocity>
          </limit>
          <dynamics>
            <damping>2.0</damping>
            <friction>1.0</friction>
          </dynamics>
        </axis>
      </joint>
      
      <!-- Add plugin for ROS control -->
      <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/simple_humanoid</robotNamespace>
      </plugin>
    </model>
  </world>
</sdf>
```

</TabItem>
<TabItem value="physics_config" label="Physics Configuration Node">

```python
#!/usr/bin/env python3

"""
Node to dynamically configure physics parameters in Gazebo
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from std_srvs.srv import Empty


class PhysicsConfigurator(Node):

    def __init__(self):
        super().__init__('physics_configurator')
        
        # Create clients for physics services
        self.get_physics_client = self.create_client(
            GetPhysicsProperties, 
            '/gazebo/get_physics_properties'
        )
        self.set_physics_client = self.create_client(
            SetPhysicsProperties, 
            '/gazebo/set_physics_properties'
        )
        self.pause_physics_client = self.create_client(
            Empty, 
            '/gazebo/pause_physics'
        )
        self.unpause_physics_client = self.create_client(
            Empty, 
            '/gazebo/unpause_physics'
        )
        
        # Wait for services
        while not self.get_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_physics_properties service...')
        
        while not self.set_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_physics_properties service...')
        
        # Configure physics initially
        self.timer = self.create_timer(5.0, self.configure_physics)
        
        self.get_logger().info('Physics configurator initialized')

    def configure_physics(self):
        """Configure physics properties for humanoid simulation"""
        try:
            # Get current physics properties
            get_request = GetPhysicsProperties.Request()
            get_future = self.get_physics_client.call_async(get_request)
            
            # Wait for response
            rclpy.spin_until_future_complete(self, get_future)
            response = get_future.result()
            
            if response is None:
                self.get_logger().error('Failed to get physics properties')
                return
            
            self.get_logger().info(f'Current gravity: {response.gravity}')
            self.get_logger().info(f'Current step size: {response.time_step}')
            
            # Modify properties for humanoid simulation
            set_request = SetPhysicsProperties.Request()
            set_request.time_step = 0.001  # 1ms for stable humanoid simulation
            set_request.max_update_rate = 1000.0
            set_request.gravity = response.gravity  # Keep Earth gravity
            set_request.gravity.z = -9.8  # Ensure proper direction
            
            # ODE physics parameters
            set_request.ode_config = ODEPhysics()
            set_request.ode_config.auto_disable_bodies = False
            set_request.ode_config.sor_pgs_precon_iters = 0
            set_request.ode_config.sor_pgs_iters = 50
            set_request.ode_config.sor_pgs_w = 1.3
            set_request.ode_config.sor_pgs_rms_error_tol = 0.01
            set_request.ode_config.contact_surface_layer = 0.001
            set_request.ode_config.contact_max_correcting_vel = 100.0
            set_request.ode_config.cfm = 0.0
            set_request.ode_config.erp = 0.2
            set_request.ode_config.max_contacts = 20
            
            # Pause physics while updating
            pause_request = Empty.Request()
            pause_future = self.pause_physics_client.call_async(pause_request)
            rclpy.spin_until_future_complete(self, pause_future)
            
            # Set new physics properties
            set_future = self.set_physics_client.call_async(set_request)
            rclpy.spin_until_future_complete(self, set_future)
            
            if set_future.result() is not None and set_future.result().success:
                self.get_logger().info('Physics properties updated successfully')
                
                # Print new settings
                self.get_logger().info(f'New time step: {set_request.time_step}')
                self.get_logger().info(f'New max update rate: {set_request.max_update_rate}')
                
            else:
                self.get_logger().error('Failed to set physics properties')
            
            # Resume physics
            unpause_request = Empty.Request()
            unpause_future = self.unpause_physics_client.call_async(unpause_request)
            rclpy.spin_until_future_complete(self, unpause_future)
            
        except Exception as e:
            self.get_logger().error(f'Error configuring physics: {e}')


def main(args=None):
    rclpy.init(args=args)

    physics_configurator = PhysicsConfigurator()

    try:
        rclpy.spin(physics_configurator)
    except KeyboardInterrupt:
        physics_configurator.get_logger().info('Physics configurator stopped by user')
    finally:
        physics_configurator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
<TabItem value="physics_tuning" label="Physics Tuning for Humanoid">

```python
#!/usr/bin/env python3

"""
Script to tune physics parameters for humanoid robot simulation
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelConfiguration, GetModelState
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64
import time
import math


class HumanoidPhysicsTuner(Node):

    def __init__(self):
        super().__init__('humanoid_physics_tuner')
        
        # Robot state publisher
        self.com_publisher = self.create_publisher(
            Point, 
            '/humanoid/com', 
            10
        )
        
        # Physics parameters publisher
        self.damping_publisher = self.create_publisher(
            Float64, 
            '/humanoid/damping', 
            10
        )
        
        # Timer to check robot stability
        self.stability_timer = self.create_timer(0.1, self.check_stability)
        
        # Robot state subscriber
        self.robot_state_subscriber = self.create_subscription(
            Point,
            '/humanoid/com',
            self.com_callback,
            10
        )
        
        # Initialize parameters
        self.robot_name = "simple_humanoid"
        self.com_history = []
        self.max_history = 100  # Store last 10 seconds of CoM data
        
        self.get_logger().info('Humanoid Physics Tuner initialized')

    def com_callback(self, msg):
        """Receive center of mass data"""
        self.com_history.append((self.get_clock().now().nanoseconds, msg.x, msg.y, msg.z))
        
        # Keep only recent history
        current_time = self.get_clock().now().nanoseconds
        self.com_history = [item for item in self.com_history 
                           if current_time - item[0] < 10 * 1e9]  # 10 seconds

    def calculate_stability_metrics(self):
        """Calculate stability metrics from CoM history"""
        if len(self.com_history) < 10:
            return None, None, None  # Need more data
        
        # Calculate CoM movement statistics
        z_values = [item[3] for item in self.com_history]  # z position
        x_values = [item[1] for item in self.com_history]  # x position
        
        avg_z = sum(z_values) / len(z_values)
        avg_x = sum(x_values) / len(x_values)
        
        # Calculate variance (stability indicator)
        z_variance = sum((z - avg_z)**2 for z in z_values) / len(z_values)
        x_variance = sum((x - avg_x)**2 for x in x_values) / len(x_values)
        
        return avg_z, z_variance, x_variance

    def check_stability(self):
        """Monitor robot stability and adjust physics parameters"""
        avg_z, z_var, x_var = self.calculate_stability_metrics()
        
        if avg_z is not None:
            # Calculate stability index (lower is more stable)
            stability_index = (z_var + x_var) * 100
            
            self.get_logger().info(f'Stability: {stability_index:.2f}, '
                                  f'Z variance: {z_var:.4f}, X variance: {x_var:.4f}')
            
            # If instability detected, adjust parameters
            if stability_index > 0.5:  # Threshold for instability
                self.get_logger().warn('Instability detected, adjusting parameters')
                self.adjust_physics_parameters()
            
            # Publish stability metrics
            stability_msg = Float64()
            stability_msg.data = stability_index
            self.damping_publisher.publish(stability_msg)

    def adjust_physics_parameters(self):
        """Dynamically adjust physics parameters based on stability"""
        # In a real implementation, this would call Gazebo services
        # to modify joint damping, friction, etc.
        self.get_logger().info('Adjusting physics parameters for stability')
        
        # Example: Increase damping to improve stability
        # This would typically be done via joint controllers in real implementation
        new_damping = 2.5  # Increased damping value
        
        self.get_logger().info(f'Increasing joint damping to {new_damping}')


def main(args=None):
    rclpy.init(args=args)

    physics_tuner = HumanoidPhysicsTuner()

    try:
        rclpy.spin(physics_tuner)
    except KeyboardInterrupt:
        physics_tuner.get_logger().info('Physics tuner stopped by user')
    finally:
        physics_tuner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</TabItem>
</Tabs>

Expected Output:
```
[INFO] [1678882844.123456789] [physics_configurator]: Physics configurator initialized
[INFO] [1678882844.123456789] [physics_configurator]: Current gravity: x: 0.0, y: 0.0, z: -9.8
[INFO] [1678882844.123456789] [physics_configurator]: Current step size: 0.001
[INFO] [1678882844.123456789] [physics_configurator]: Physics properties updated successfully
[INFO] [1678882844.123456789] [physics_configurator]: New time step: 0.001
[INFO] [1678882844.123456789] [physics_configurator]: New max update rate: 1000.0

[INFO] [1678882845.123456789] [humanoid_physics_tuner]: Humanoid Physics Tuner initialized
[INFO] [1678882845.223456789] [humanoid_physics_tuner]: Stability: 0.12, Z variance: 0.0005, X variance: 0.0007
[INFO] [1678882845.323456789] [humanoid_physics_tuner]: Stability: 0.11, Z variance: 0.0004, X variance: 0.0007
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Stability Tuning**: Adjust physics parameters to improve robot stability
   - [ ] Modify damping values to reduce oscillations
   - [ ] Adjust friction coefficients to prevent slipping
   - [ ] Test how different values affect robot locomotion
   - [ ] Find optimal values for your specific robot model

2. **Collision Detection**: Configure collision properties for a complex robot
   - [ ] Set up collision geometry for all robot links
   - [ ] Test collision behavior with different objects
   - [ ] Adjust collision parameters to prevent interpenetration
   - [ ] Validate that collisions are detected and handled properly

## Common Pitfalls and Solutions

- **Pitfall 1**: Simulation instability - Robot behaving erratically or exploding
  - *Solution*: Reduce time step, increase iterations, or add damping to joints
- **Pitfall 2**: Performance issues - Slow simulation with complex physics
  - *Solution*: Simplify collision meshes, reduce update rate, or adjust solver parameters
- **Pitfall 3**: Unrealistic behavior - Robot moving unlike real hardware
  - *Solution*: Fine-tune mass properties, friction, and damping values
- **Pitfall 4**: Interpenetration - Objects passing through each other
  - *Solution*: Increase contact stiffness, reduce time step, or improve collision geometry

## Summary

- Physics engines govern how objects interact in simulation
- Gravity, friction, and damping are critical for realistic behavior
- Proper mass and inertia properties ensure accurate dynamics
- Physics parameters must be tuned for humanoid robots specifically
- Realistic physics is essential for sim-to-real transfer

## Further Reading

- [Gazebo Physics Documentation](http://gazebosim.org/tutorials?tut=physics)
- [ODE Parameter Tuning](http://gazebosim.org/tutorials?tut=physics_ode_tuning)
- [Collision Detection Tutorial](http://gazebosim.org/tutorials?tut=shapes)
- [Humanoid Robotics Physics](https://ieeexplore.ieee.org/document/6225589)