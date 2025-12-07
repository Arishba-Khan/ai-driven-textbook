---
title: "URDF to SDF Conversion"
description: "Converting URDF robot models for Gazebo simulation"
---

## Learning Objectives

After completing this chapter, you will be able to:
- Convert URDF robot descriptions to SDF for Gazebo simulation
- Add Gazebo-specific extensions to robot models
- Configure physics properties for accurate simulation
- Integrate sensors and actuators in simulated robots

## Introduction

The conversion from URDF (Unified Robot Description Format) to SDF (Simulation Description Format) is a critical step in preparing robots for Gazebo simulation. While URDF is the standard format for describing robots in ROS, Gazebo operates with SDF, which includes additional elements and properties specifically designed for physics simulation and rendering. Understanding this conversion process is essential for creating accurate and functional robot simulations in the Physical AI & Humanoid Robotics pipeline.

The conversion process involves not just translating the kinematic structure but also adding simulation-specific elements like collision properties, friction parameters, and sensor configurations. For humanoid robots with complex kinematics, this process becomes even more important as their many degrees of freedom require careful physics modeling to ensure realistic behavior in simulation.

## Core Concepts

URDF and SDF are both XML-based formats but serve different purposes. URDF focuses on the geometric and kinematic description of robots, while SDF extends this to include physics properties, visual effects, and simulation-specific elements. The conversion process typically involves:

### URDF to SDF Translation

The basic elements of a robot (links, joints) translate directly from URDF to SDF. However, SDF requires additional information for simulation, such as material properties, friction coefficients, and collision parameters.

### Gazebo-Specific Extensions

URDF supports Gazebo-specific extensions using the `<gazebo>` tag. These extensions allow you to add simulation-specific properties without changing the core URDF, which may be used by real hardware controllers as well.

### Physics Configuration

For accurate simulation, physics properties like mass, inertia, friction, and damping must be properly defined. For humanoid robots, these properties are critical for achieving realistic movement patterns and ensuring stability during simulation.

## Hands-on Examples

Let's demonstrate the URDF to SDF conversion process:

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="urdf_model" label="Original URDF Model" default>

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.2"/>
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

  <!-- Example leg joint -->
  <joint name="torso_to_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_hip"/>
    <origin xyz="-0.1 0.0 -0.25"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="1.0" friction="0.1"/>
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

  <!-- Gazebo-specific extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="torso">
    <material>Gazebo/Grey</material>
  </gazebo>
  
  <gazebo reference="left_hip">
    <material>Gazebo/Red</material>
  </gazebo>
  
  <!-- Gazebo controller plugin -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_humanoid</robotNamespace>
    </plugin>
  </gazebo>
</robot>
```

</TabItem>
<TabItem value="converted_sdf" label="Converted SDF Model">

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_humanoid">
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.1 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
          <specular>0 0 0.8 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.1 0.2</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.1</iyy>
          <iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
    </link>

    <link name="torso">
      <pose>0 0 0.15 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>15.0</mass>
        <inertia>
          <ixx>0.2</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.2</iyy>
          <iyz>0.0</iyz>
          <izz>0.2</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="base_to_torso" type="fixed">
      <parent>base_link</parent>
      <child>torso</child>
      <pose>0 0 0.15 0 0 0</pose>
    </joint>

    <link name="left_hip">
      <pose>-0.1 0.0 -0.25 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.15</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0 0 1</ambient>
          <diffuse>0.8 0 0 1</diffuse>
          <specular>0.8 0 0 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.15</length>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>3.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="torso_to_hip" type="revolute">
      <parent>torso</parent>
      <child>left_hip</child>
      <pose>-0.1 0.0 -0.25 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>100.0</effort>
          <velocity>1.0</velocity>
        </limit>
        <dynamics>
          <damping>1.0</damping>
          <friction>0.1</friction>
        </dynamics>
      </axis>
    </joint>

    <!-- Add Gazebo-specific physics properties -->
    <static>false</static>
    <self_collide>false</self_collide>
    <enable_wind>false</enable_wind>
    <kinematic>false</kinematic>

    <!-- Add plugin for ROS control -->
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_humanoid</robotNamespace>
    </plugin>
  </model>
</sdf>
```

</TabItem>
<TabItem value="conversion_script" label="Conversion Script">

```python
#!/usr/bin/env python3

"""
Script to convert URDF to SDF with Gazebo-specific extensions
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
import argparse
import os


def urdf_to_sdf(urdf_file_path, sdf_file_path):
    """
    Convert a URDF file to SDF format with Gazebo-specific extensions
    """
    # Parse the URDF
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()
    
    # Create SDF root
    sdf = ET.Element("sdf", version="1.7")
    model = ET.SubElement(sdf, "model", name=root.get("name", "converted_model"))
    
    # Process links
    for link in root.findall("link"):
        link_name = link.get("name")
        sdf_link = ET.SubElement(model, "link", name=link_name)
        
        # Process visual element
        visual = link.find("visual")
        if visual is not None:
            sdf_visual = ET.SubElement(sdf_link, "visual", name="visual")
            
            # Copy geometry
            urdf_geometry = visual.find("geometry")
            if urdf_geometry is not None:
                sdf_geometry = ET.SubElement(sdf_visual, "geometry")
                
                # Copy geometry type (box, cylinder, sphere, mesh)
                for geom_type in ["box", "cylinder", "sphere", "mesh"]:
                    geom_element = urdf_geometry.find(geom_type)
                    if geom_element is not None:
                        sdf_geom = ET.SubElement(sdf_geometry, geom_type)
                        for attr in geom_element.attrib:
                            sdf_geom.set(attr, geom_element.get(attr))
                        break
            
            # Copy material
            material = visual.find("material")
            if material is not None:
                sdf_material = ET.SubElement(sdf_visual, "material")
                
                color = material.find("color")
                if color is not None:
                    rgba = color.get("rgba", "0.5 0.5 0.5 1.0").split()
                    rgba = [float(x) for x in rgba]
                    
                    ambient = ET.SubElement(sdf_material, "ambient")
                    ambient.text = f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"
                    
                    diffuse = ET.SubElement(sdf_material, "diffuse")
                    diffuse.text = f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"
        
        # Process collision element
        collision = link.find("collision")
        if collision is not None:
            sdf_collision = ET.SubElement(sdf_link, "collision", name="collision")
            
            # Copy geometry
            urdf_geometry = collision.find("geometry")
            if urdf_geometry is not None:
                sdf_geometry = ET.SubElement(sdf_collision, "geometry")
                
                # Copy geometry type
                for geom_type in ["box", "cylinder", "sphere", "mesh"]:
                    geom_element = urdf_geometry.find(geom_type)
                    if geom_element is not None:
                        sdf_geom = ET.SubElement(sdf_geometry, geom_type)
                        for attr in geom_element.attrib:
                            sdf_geom.set(attr, geom_element.get(attr))
                        break
        
        # Process inertial element
        inertial = link.find("inertial")
        if inertial is not None:
            sdf_inertial = ET.SubElement(sdf_link, "inertial")
            
            mass = inertial.find("mass")
            if mass is not None:
                sdf_mass = ET.SubElement(sdf_inertial, "mass")
                sdf_mass.text = mass.get("value", "0.1")
            
            inertia = inertial.find("inertia")
            if inertia is not None:
                sdf_inertia = ET.SubElement(sdf_inertial, "inertia")
                
                for attr in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
                    value = inertia.get(attr, "0.001")
                    sdf_attr = ET.SubElement(sdf_inertia, attr)
                    sdf_attr.text = value
    
    # Process joints
    for joint in root.findall("joint"):
        joint_name = joint.get("name")
        joint_type = joint.get("type")
        
        sdf_joint = ET.SubElement(model, "joint", name=joint_name, type=joint_type)
        
        # Copy parent and child
        parent = joint.find("parent")
        if parent is not None:
            sdf_parent = ET.SubElement(sdf_joint, "parent")
            sdf_parent.text = parent.get("link")
        
        child = joint.find("child")
        if child is not None:
            sdf_child = ET.SubElement(sdf_joint, "child")
            sdf_child.text = child.get("link")
        
        # Copy axis for revolute and continuous joints
        if joint_type in ["revolute", "continuous", "prismatic"]:
            axis = joint.find("axis")
            if axis is not None:
                sdf_axis = ET.SubElement(sdf_joint, "axis")
                
                xyz = axis.get("xyz", "1 0 0")
                sdf_xyz = ET.SubElement(sdf_axis, "xyz")
                sdf_xyz.text = xyz
                
                # Copy limits if present
                limit = joint.find("limit")
                if limit is not None:
                    sdf_limit = ET.SubElement(sdf_axis, "limit")
                    
                    for attr in ["lower", "upper", "effort", "velocity"]:
                        value = limit.get(attr)
                        if value:
                            sdf_attr = ET.SubElement(sdf_limit, attr)
                            sdf_attr.text = value
    
    # Add default properties
    static = ET.SubElement(model, "static")
    static.text = "false"
    
    # Write SDF to file
    rough_string = ET.tostring(sdf, encoding="unicode")
    reparsed = minidom.parseString(rough_string)
    pretty_xml = reparsed.toprettyxml(indent="  ")
    
    # Remove extra blank lines
    lines = pretty_xml.split('\n')
    filtered_lines = [line for line in lines if line.strip() != '']
    pretty_xml = '\n'.join(filtered_lines)
    
    with open(sdf_file_path, 'w') as f:
        f.write(pretty_xml)
    
    print(f"Converted {urdf_file_path} to {sdf_file_path}")


def main():
    parser = argparse.ArgumentParser(description='Convert URDF to SDF')
    parser.add_argument('urdf_file', help='Input URDF file')
    parser.add_argument('sdf_file', help='Output SDF file')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.urdf_file):
        print(f"Error: URDF file {args.urdf_file} does not exist")
        return
    
    urdf_to_sdf(args.urdf_file, args.sdf_file)


if __name__ == '__main__':
    main()
```

</TabItem>
</Tabs>

Expected Output:
```
Converted robot.urdf to robot.sdf
```

## Exercises

Complete the following exercises to reinforce your understanding:

1. **Complete Robot Model**: Convert a full humanoid robot URDF to SDF
   - [ ] Extend the example to include all humanoid joints (arms, legs, head)
   - [ ] Add appropriate physical properties for each link
   - [ ] Validate that the converted SDF works in Gazebo
   - [ ] Test that the robot maintains its kinematic structure

2. **Sensor Integration**: Add simulated sensors to the converted robot
   - [ ] Add camera sensors to the robot's head
   - [ ] Include IMU sensors on the torso
   - [ ] Add contact sensors to feet for balance control
   - [ ] Verify that sensor data is published correctly in ROS 2

## Common Pitfalls and Solutions

- **Pitfall 1**: Incorrect mass properties - Using unrealistic mass or inertia values
  - *Solution*: Calculate values based on CAD models or estimate based on volume and material density
- **Pitfall 2**: Missing collision geometry - Links without collision elements causing physics errors
  - *Solution*: Ensure every visual element has a corresponding collision element
- **Pitfall 3**: Joint limit mismatches - Joint limits in URDF not matching SDF
  - *Solution*: Carefully convert all joint limits and safety margins
- **Pitfall 4**: Controller plugin issues - ROS control not working in simulation
  - *Solution*: Verify plugin configuration and namespace matches ROS nodes

## Summary

- URDF to SDF conversion is essential for Gazebo simulation
- Gazebo extensions in URDF allow simulation-specific properties
- Physics properties must be carefully defined for realistic behavior
- Conversion scripts can automate the process for standard robots
- Validation is critical to ensure proper simulation behavior

## Further Reading

- [URDF to SDF Conversion Guide](http://gazebosim.org/tutorials?tut=ros_gzplugins)
- [SDF Documentation](http://sdformat.org/spec)
- [Gazebo Robot Plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins)
- [Physics Parameter Tuning](http://gazebosim.org/tutorials?tut=physics)