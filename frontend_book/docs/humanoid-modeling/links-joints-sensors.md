---
sidebar_position: 2
title: Links, Joints, and Sensors
description: "Learn about the fundamental building blocks of URDF: links, joints, and sensors that form the structure of robot models."
keywords: [urdf, links, joints, sensors, robot-modeling, xml]
---

# Links, Joints, and Sensors in URDF

This article covers the fundamental building blocks of URDF: links, joints, and sensors that form the structure of robot models.

## Learning Objectives

By the end of this article, you will be able to:
- Define and create links in URDF
- Understand different joint types and their properties
- Add sensors to robot models
- Structure a basic URDF model with proper connections

## Introduction to URDF Structure

URDF (Unified Robot Description Format) is an XML-based format that describes robot physical properties. A robot model in URDF consists of:

- **Links**: Rigid bodies that make up the robot structure
- **Joints**: Connections between links that allow relative motion
- **Sensors**: Elements that represent sensor components
- **Materials**: Visual properties for rendering

## Links

Links represent rigid bodies in the robot. Each link can have visual, collision, inertial, and other properties.

### Basic Link Structure:

```xml
<link name="link_name">
  <visual>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
</link>
```

### Link Components:

1. **Visual**: How the link appears in simulation and visualization
2. **Collision**: How the link interacts in collision detection
3. **Inertial**: Physical properties for dynamics simulation

### Geometry Types:

- **Box**: `<box size="width length height"/>`
- **Cylinder**: `<cylinder radius="r" length="l"/>`
- **Sphere**: `<sphere radius="r"/>`
- **Mesh**: `<mesh filename="path/to/mesh.stl"/>`

## Joints

Joints connect links and define how they can move relative to each other.

### Basic Joint Structure:

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <axis xyz="x y z"/>
  <limit lower="lower_limit" upper="upper_limit" effort="max_effort" velocity="max_velocity"/>
</joint>
```

### Joint Types:

1. **Revolute**: Rotational joint with limited range
   ```xml
   <joint name="hinge_joint" type="revolute">
     <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
   </joint>
   ```

2. **Continuous**: Rotational joint without limits
   ```xml
   <joint name="continuous_joint" type="continuous">
   </joint>
   ```

3. **Prismatic**: Linear sliding joint
   ```xml
   <joint name="slider_joint" type="prismatic">
     <limit lower="0" upper="0.5" effort="100" velocity="1"/>
   </joint>
   ```

4. **Fixed**: No movement (rigid connection)
   ```xml
   <joint name="fixed_joint" type="fixed">
   </joint>
   ```

5. **Floating**: 6 DOF with no limits
   ```xml
   <joint name="floating_joint" type="floating">
   </joint>
   ```

6. **Planar**: Movement in a plane
   ```xml
   <joint name="planar_joint" type="planar">
   </joint>
   ```

## Sensors

Sensors represent sensor components in the robot model, which can be used for simulation purposes.

### Basic Sensor Structure:

```xml
<gazebo reference="link_name">
  <sensor name="sensor_name" type="sensor_type">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>link_name</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Common Sensor Types:
- **Camera**: Visual sensors
- **LIDAR**: Range sensors
- **IMU**: Inertial measurement units
- **Force/Torque**: Force and torque sensors

## Complete Example: Simple Robot Arm

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- End effector link -->
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Fixed joint for end effector -->
  <joint name="arm_to_end_effector" type="fixed">
    <parent link="arm_link"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
</robot>
```

## Best Practices

### Link Naming:
- Use descriptive names that reflect the part's function
- Follow a consistent naming convention
- Use underscores to separate words

### Joint Configuration:
- Always specify appropriate limits for revolute joints
- Set realistic effort and velocity limits
- Use proper origin transformations

### Visual vs Collision:
- Use simple geometries for collision to improve performance
- Use detailed meshes for visual representation
- Ensure both visual and collision properties are defined

### Inertial Properties:
- Calculate accurate inertial properties for dynamics simulation
- Use tools like CAD software to calculate inertial tensors
- Set appropriate mass values

## URDF Validation

Always validate your URDF files to ensure they are properly structured:

- Check for proper XML syntax
- Ensure all joints have defined parent and child links
- Verify that all referenced links exist
- Validate that joint limits are appropriate

## Summary

Links, joints, and sensors form the fundamental building blocks of URDF robot models. Understanding how to properly define these elements is crucial for creating accurate and functional robot models that can be used in both simulation and real robot control. Each component has specific properties that determine how the robot behaves in different contexts.

## Navigation

- **Previous**: [Humanoid Modeling with URDF](./intro.md)
- **Next**: [Representing Anatomy](./representing-anatomy.md)