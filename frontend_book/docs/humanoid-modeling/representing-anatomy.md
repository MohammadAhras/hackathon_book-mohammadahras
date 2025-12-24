---
sidebar_position: 3
title: Representing Anatomy
description: Explore how to model humanoid structures using URDF, focusing on the representation of human-like anatomy in robot models.
keywords: [urdf, humanoid, anatomy, modeling, robot-structure, biomechanics]
---

# Representing Humanoid Anatomy in URDF

This article explores how to model humanoid structures using URDF, focusing on the representation of human-like anatomy in robot models.

## Learning Objectives

By the end of this article, you will be able to:
- Model humanoid body parts using appropriate URDF structures
- Create articulated joints that mimic human movement
- Design realistic proportions for humanoid robots
- Understand the biomechanical considerations in humanoid modeling

## Humanoid Robot Anatomy Overview

Humanoid robots are designed to have a human-like structure with similar degrees of freedom. A typical humanoid robot includes:

- **Torso**: The central body structure
- **Head**: Contains sensors and cameras
- **Arms**: With shoulder, elbow, and wrist joints
- **Hands**: With finger articulation (optional)
- **Legs**: With hip, knee, and ankle joints
- **Feet**: For balance and locomotion

## Modeling the Torso

The torso serves as the central structure connecting all other body parts:

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.2 0.1 0.4"/>  <!-- Width, depth, height -->
    </geometry>
    <material name="torso_material">
      <color rgba="0.8 0.8 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.1 0.4"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

### Torso Considerations:
- Should be the main reference frame for the robot
- Contains electronics and power systems in real robots
- Needs sufficient size to connect multiple limbs

## Modeling the Head

The head typically contains sensors and is connected to the torso:

```xml
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
    <material name="head_material">
      <color rgba="0.9 0.9 0.9 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
  </inertial>
</link>

<!-- Neck joint -->
<joint name="neck_joint" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Pitch motion -->
  <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
</joint>
```

## Modeling Arms

Humanoid arms typically have 7 degrees of freedom each (shoulder: 3, elbow: 1, wrist: 3):

```xml
<!-- Shoulder links -->
<link name="left_shoulder">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
    <material name="arm_material">
      <color rgba="0.6 0.6 0.6 1"/>
    </material>
  </visual>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Shoulder joints -->
<joint name="left_shoulder_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_shoulder"/>
  <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Yaw motion -->
  <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
</joint>

<joint name="left_shoulder_pitch" type="revolute">
  <parent link="left_shoulder"/>
  <child link="left_upper_arm"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Pitch motion -->
  <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
</joint>

<joint name="left_shoulder_roll" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_lower_arm"/>
  <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Roll motion -->
  <limit lower="-2.0" upper="2.0" effort="15" velocity="1"/>
</joint>

<!-- Elbow joint -->
<joint name="left_elbow" type="revolute">
  <parent link="left_lower_arm"/>
  <child link="left_hand"/>
  <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Elbow flexion -->
  <limit lower="0" upper="2.5" effort="15" velocity="1"/>
</joint>
```

### Arm Design Considerations:
- Should provide sufficient reach for intended tasks
- Joint limits should reflect human-like capabilities
- Consider workspace requirements for manipulation tasks

## Modeling Legs

Humanoid legs typically have 6 degrees of freedom each (hip: 3, knee: 1, ankle: 2):

```xml
<!-- Hip joint -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_hip"/>
  <origin xyz="0.05 -0.05 -0.2" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Hip yaw -->
  <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
</joint>

<joint name="left_hip_roll" type="revolute">
  <parent link="left_hip"/>
  <child link="left_thigh"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Hip roll -->
  <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
</joint>

<joint name="left_hip_pitch" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Hip pitch -->
  <limit lower="-1.57" upper="0.5" effort="50" velocity="1"/>
</joint>

<!-- Knee joint -->
<joint name="left_knee" type="revolute">
  <parent link="left_shin"/>
  <child link="left_foot"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Knee flexion -->
  <limit lower="0" upper="2.0" effort="50" velocity="1"/>
</joint>

<!-- Ankle joints -->
<joint name="left_ankle_pitch" type="revolute">
  <parent link="left_foot"/>
  <child link="left_foot_tip"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Ankle pitch -->
  <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
</joint>

<joint name="left_ankle_roll" type="revolute">
  <parent link="left_foot_tip"/>
  <child link="left_sole"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Ankle roll -->
  <limit lower="-0.3" upper="0.3" effort="20" velocity="1"/>
</joint>
```

### Leg Design Considerations:
- Must support the robot's weight during locomotion
- Joint limits should allow for stable walking patterns
- Foot design affects balance and ground contact

## Biomechanical Considerations

### Joint Ranges of Motion:
- Shoulder: ~180° in multiple directions
- Elbow: ~150° flexion
- Hip: ~120° flexion, ~45° extension
- Knee: ~150° flexion
- Ankle: ~50° dorsiflexion, ~20° plantarflexion

### Proportional Relationships:
- Arm length: ~0.4 * height
- Leg length: ~0.5 * height
- Torso length: ~0.3 * height

### Center of Mass:
- Should be within the support polygon during standing
- Dynamic balance during locomotion
- Consider mass distribution for stability

## Advanced Anatomical Features

### Flexible Spine:
```xml
<!-- Multi-joint spine for flexibility -->
<joint name="spine_yaw" type="revolute">
  <parent link="torso_base"/>
  <child link="torso_upper"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.3" upper="0.3" effort="30" velocity="1"/>
</joint>

<joint name="spine_pitch" type="revolute">
  <axis xyz="0 1 0"/>
  <limit lower="-0.3" upper="0.3" effort="30" velocity="1"/>
</joint>
```

### Expressive Head:
```xml
<!-- Eyes for visual appeal -->
<link name="left_eye">
  <visual>
    <geometry>
      <sphere radius="0.01"/>
    </geometry>
    <material name="eye_material">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
</link>
```

## Integration with AI and Control

### Sensor Integration:
- IMU in torso for balance control
- Force/torque sensors in joints
- Cameras in head for vision
- Tactile sensors in hands/feet

### Control Considerations:
- Joint limits for safety
- Redundant solutions for complex movements
- Inverse kinematics for task execution

## Example: Complete Simple Humanoid

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.15 0.1 0.3"/>
      </geometry>
      <material name="gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>
  </link>

  <joint name="neck" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1"/>
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.1 0.05 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
  </joint>
</robot>
```

## Summary

Modeling humanoid anatomy in URDF requires careful consideration of human-like proportions, joint configurations, and biomechanical constraints. By properly representing the torso, head, arms, and legs with appropriate links and joints, you can create robot models that are suitable for both simulation and real-world control applications. Understanding these anatomical principles is essential for developing humanoid robots that can effectively interact with human environments and perform complex tasks.

## Navigation

- **Previous**: [Links, Joints, and Sensors](./links-joints-sensors.md)
- **Next**: [URDF's Role in Simulation and Control](./urdf-simulation-control.md)