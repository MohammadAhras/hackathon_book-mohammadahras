---
sidebar_position: 3
title: World and Robot Simulation Basics
description: Learn about creating and configuring simulation environments with proper physics properties.
keywords: [gazebo, world, simulation, robot, environment, physics, robotics]
---

# World and Robot Simulation Basics

This article covers creating and configuring simulation environments with proper physics properties, including world setup and robot integration in Gazebo.

## Learning Objectives

By the end of this article, you will be able to:
- Create basic simulation worlds with proper physics properties
- Configure environment elements for realistic simulation
- Integrate robots into simulation environments
- Set up proper physics and lighting parameters

## World Simulation Fundamentals

A simulation world in Gazebo is defined by an SDF (Simulation Description Format) file that specifies the environment, physics properties, lighting, and initial object placements. Understanding world simulation basics is essential for creating effective digital twins.

### World File Structure

A basic Gazebo world file follows this structure:

```xml
<sdf version='1.7'>
  <world name='my_world'>
    <!-- Physics engine configuration -->
    <physics name='default' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting configuration -->
    <light name='sun' type='directional'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Ambient light -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
    </scene>

    <!-- Plugins and models -->
    <!-- Additional world elements -->
  </world>
</sdf>
```

## Physics Configuration

### Physics Engine Parameters

The physics engine controls how objects behave in the simulation. Key parameters include:

#### Time Step Settings
```xml
<physics name='default' type='ode'>
  <max_step_size>0.001</max_step_size>  <!-- Simulation time step (seconds) -->
  <real_time_factor>1</real_time_factor>  <!-- Real-time factor (1 = real-time) -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- Hz -->
</physics>
```

- **max_step_size**: Smaller values provide more accuracy but require more computation
- **real_time_factor**: Ratio of simulation time to real time (1.0 = real-time)
- **real_time_update_rate**: Rate at which physics updates occur

#### Solver Parameters
```xml
<ode>
  <solver>
    <type>quick</type>  <!-- or "world" -->
    <iters>10</iters>    <!-- Number of solver iterations -->
    <sor>1.3</sor>      <!-- Successive over-relaxation parameter -->
  </solver>
  <constraints>
    <cfm>0</cfm>        <!-- Constraint Force Mixing -->
    <erp>0.2</erp>      <!-- Error Reduction Parameter -->
    <contact_max_correcting_vel>100</contact_max_correcting_vel>
    <contact_surface_layer>0.001</contact_surface_layer>
  </constraints>
</ode>
```

## Environment Elements

### Creating Custom Environments

Custom environments can be created using SDF models or by modifying existing ones. Here's an example of creating a simple room:

```xml
<model name='room'>
  <pose>0 0 0 0 0 0</pose>
  <static>true</static>

  <!-- Floor -->
  <link name='floor'>
    <collision>
      <geometry>
        <box>
          <size>10 10 0.1</size>
        </box>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <box>
          <size>10 10 0.1</size>
        </box>
      </geometry>
    </visual>
  </link>

  <!-- Walls -->
  <link name='wall_north'>
    <pose>0 5 2.5 0 0 0</pose>
    <collision>
      <geometry>
        <box>
          <size>10 0.1 5</size>
        </box>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <box>
          <size>10 0.1 5</size>
        </box>
      </geometry>
    </visual>
  </link>

  <!-- Additional walls... -->
</model>
```

### Terrain and Outdoor Environments

For outdoor simulations, terrain models can be created:

```xml
<model name='terrain'>
  <static>true</static>
  <link name='terrain_link'>
    <collision>
      <geometry>
        <heightmap>
          <uri>file://path/to/heightmap.png</uri>
          <size>100 100 20</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <heightmap>
          <uri>file://path/to/texture.png</uri>
          <size>100 100 20</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

## Robot Integration

### Loading Robots into Worlds

Robots can be loaded into simulation worlds in several ways:

#### Method 1: Direct inclusion in world file
```xml
<include>
  <uri>model://my_robot</uri>
  <pose>1 1 0.5 0 0 0</pose>  <!-- x, y, z, roll, pitch, yaw -->
</include>
```

#### Method 2: Spawn model via command line
```bash
gz model -f /path/to/robot.urdf -m robot_name -x 1.0 -y 1.0 -z 0.5
```

#### Method 3: Spawn via ROS service call
```python
import rospy
from gazebo_msgs.srv import SpawnModel

rospy.wait_for_service('/gazebo/spawn_urdf_model')
spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

with open('/path/to/robot.urdf', 'r') as f:
    robot_xml = f.read()

spawn_model(model_name='my_robot',
           model_xml=robot_xml,
           robot_namespace='/',
           initial_pose=pose,
           reference_frame='world')
```

### Robot-Environment Interaction

The key to effective robot-environment interaction is ensuring proper physics properties on both the robot and environment:

```xml
<!-- Robot model with appropriate physics properties -->
<link name="wheel_link">
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
  </inertial>
  <collision>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>   <!-- Forward friction coefficient -->
          <mu2>0.8</mu2> <!-- Side friction coefficient -->
        </ode>
      </friction>
    </surface>
  </collision>
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
  </visual>
</link>
```

## Lighting and Rendering

### Light Types and Configuration

Proper lighting enhances the visual quality and realism of simulation:

```xml
<!-- Directional light (like sun) -->
<light name='directional_light' type='directional'>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <specular>0.5 0.5 0.5 1</specular>
  <attenuation>
    <range>100</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.3 -0.2 -0.9</direction>
</light>

<!-- Point light -->
<light name='point_light' type='point'>
  <pose>5 5 5 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>10</range>
    <constant>0.2</constant>
    <linear>0.3</linear>
    <quadratic>0.2</quadratic>
  </attenuation>
</light>
```

### Scene Configuration

The scene element controls visual appearance:

```xml
<scene>
  <ambient>0.3 0.3 0.3 1</ambient>      <!-- Ambient light color -->
  <background>0.6 0.7 0.8 1</background> <!-- Background color -->
  <shadows>true</shadows>               <!-- Enable/disable shadows -->
  <grid>false</grid>                    <!-- Show/hide grid -->
  <origin_visual>false</origin_visual>  <!-- Show/hide origin visual -->
</scene>
```

## Practical Example: Complete World Configuration

Here's a complete example of a simple indoor world for robot simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_indoor_world">
    <!-- Physics configuration -->
    <physics name="default" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <!-- Indoor environment -->
    <model name="indoor_room">
      <static>true</static>
      <link name="floor">
        <collision>
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual>
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Walls -->
      <link name="north_wall">
        <pose>0 5 2.5 0 0 0</pose>
        <collision>
          <geometry>
            <box>
              <size>10 0.1 5</size>
            </box>
          </geometry>
        </collision>
        <visual>
          <geometry>
            <box>
              <size>10 0.1 5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Additional walls... -->
    </model>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting configuration -->
    <scene>
      <ambient>0.3 0.3 0.3 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Fog effect (optional) -->
    <atmosphere type="adiabatic">
      <temperature>288.15</temperature>
      <pressure>101325</pressure>
    </atmosphere>
  </world>
</sdf>
```

## Best Practices

### World Design
1. **Start Simple**: Begin with basic shapes and gradually add complexity
2. **Realistic Physics**: Use appropriate mass, friction, and collision properties
3. **Performance**: Balance visual quality with simulation performance
4. **Consistency**: Maintain consistent units and coordinate systems

### Robot Integration
- Verify URDF/SDF compatibility with the physics engine
- Test robot mobility in the environment
- Validate sensor placement and functionality
- Check for collision issues between robot and environment

### Simulation Optimization
- Use appropriate time steps for your application
- Optimize collision geometry for performance
- Configure lighting for adequate visibility
- Validate that physics parameters match real-world expectations

## Common Issues and Solutions

### Slow Simulation
- Increase time step (but watch for instability)
- Reduce solver iterations
- Simplify collision geometry
- Limit the number of objects in the scene

### Unstable Physics
- Decrease time step
- Increase solver iterations
- Check mass and inertia values
- Verify joint limits and constraints

### Robot Falls Through Ground
- Check collision geometry alignment
- Verify that static flag is set for environment models
- Check for proper gravity settings
- Validate coordinate system alignment

## Summary

World and robot simulation basics are fundamental to creating effective digital twins in Gazebo. Proper configuration of physics properties, environment elements, and robot integration ensures realistic and stable simulation environments. Understanding these concepts allows you to create simulation worlds that accurately represent real-world scenarios for robot testing and development.

## Navigation

- **Previous**: [Gravity, Collisions, and Dynamics](./gravity-collisions-dynamics.md)
- **Next**: [Role of Gazebo in Robotics Testing](./role-gazebo-robotics-testing.md)