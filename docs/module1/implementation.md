# Module 1: ROS 2 Control Pipeline Implementation

This document details the implementation of the ROS 2 control pipeline for the humanoid robot.

## Overview

The ROS 2 control pipeline enables students to build a full control system for a humanoid robot, connecting Python agents to ROS controllers and authoring URDF models.

## Components

### URDF Model

The humanoid robot is described using a URDF (Unified Robot Description Format) file that defines the robot's physical and visual properties. The model includes:

- **Base Link**: The main body of the robot
- **Torso**: Central body section
- **Head**: With neck joint for movement
- **Arms**: Left and right arms with shoulder and elbow joints
- **Legs**: Left and right legs with hip and knee joints

### ROS 2 Nodes

The system includes several ROS 2 nodes for different functions:

#### Joint Publisher Node
- Publishes joint state information
- Controls joint positions in simulation
- Runs at 50Hz for smooth control

#### Sensor Subscriber Node
- Subscribes to sensor data from the robot
- Handles joint states, IMU data, and other sensor inputs
- Provides real-time feedback on robot status

#### Control Service Node
- Provides services for enabling/disabling the robot
- Offers reset functionality
- Manages high-level robot commands

#### Python Agent
- Demonstrates how to connect Python code to ROS controllers
- Shows how to send commands and receive sensor data
- Implements example behaviors like arm waving

## Usage

### Launching the Robot

To launch the robot in simulation:

```bash
# Launch the robot with RViz for visualization
ros2 launch humanoid_description robot.launch.py

# Launch the robot in Gazebo simulation
ros2 launch humanoid_description gazebo.launch.py
```

### Controlling the Robot

The Python agent can be run to demonstrate control:

```bash
# Run the humanoid agent
ros2 run humanoid_control agent.py
```

### Sending Commands

The system accepts various command types:

1. **Joint Trajectory Commands**: Move joints to specific positions
2. **Velocity Commands**: Control robot movement
3. **Service Calls**: Enable/disable robot, reset position

## Key Concepts Learned

Students will learn:

1. How to create and use URDF models for robot description
2. How to implement ROS 2 nodes for robot control
3. How to connect Python agents to ROS controllers
4. How to handle sensor data and robot feedback
5. How to implement basic robot behaviors

## Testing

The system can be tested by:

1. Verifying that joint states are published correctly
2. Confirming that the robot model appears correctly in RViz
3. Testing that the Python agent can control the robot
4. Validating that sensor data is received properly