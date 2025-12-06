# Module 2: Digital Twin Environment Implementation

This document details the implementation of the digital twin environment for the humanoid robot, including physics simulation and sensor data.

## Overview

The digital twin environment enables students to build realistic simulation environments for humanoid testing using Gazebo, incorporating physics simulation and sensor data.

## Components

### Gazebo World Configuration

The simulation environment is configured with a realistic world that includes:

- Physics parameters (gravity, collision detection)
- Environmental objects for testing
- Proper lighting and visual elements

### Sensor Simulation

The digital twin includes three main types of sensors:

#### LiDAR Sensor
- Mounted on top of the robot's head
- 360-degree scanning capability
- Range: 0.12m to 3.5m
- 64 ray samples for detailed environment mapping

#### Depth Camera
- Positioned on the front of the robot's head
- 640x480 resolution
- 60-degree horizontal field of view
- Capable of generating both RGB and depth images

#### IMU Sensor
- Located in the torso of the robot
- Provides orientation and acceleration data
- Includes realistic noise models for real-world simulation
- 100Hz update rate

### URDF Integration

The sensor configurations are integrated into the main humanoid URDF model using xacro macros:

- `sensors/lidar.urdf.xacro` - LiDAR sensor macro
- `sensors/camera.urdf.xacro` - Depth camera macro
- `sensors/imu.urdf.xacro` - IMU sensor macro

## Usage

### Launching the Digital Twin

To launch the robot with full sensor simulation in Gazebo:

```bash
# Launch the robot in Gazebo with all sensors
ros2 launch humanoid_description gazebo.launch.py
```

### Accessing Sensor Data

The sensors publish data on the following topics:

- LiDAR: `/scan` (sensor_msgs/LaserScan)
- Camera: `/camera/depth/image_raw` (sensor_msgs/Image)
- IMU: `/imu/data` (sensor_msgs/Imu)

### Testing Sensor Data

The system can be tested using the provided test script:

```bash
# Run the sensor simulation test
python3 ros_ws/src/humanoid_control/test/test_sensor_simulation.py
```

## Unity Environment (Conceptual)

For human-robot interaction (HRI) scenarios requiring high-fidelity 3D visualization, Unity can be used as an alternative simulation environment. However, for academic purposes, we recommend using open-source alternatives such as:

- **Gazebo Garden**: For advanced physics simulation and rendering
- **Ignition Gazebo**: For more realistic lighting and materials
- **Webots**: For web-based simulation with good 3D visualization

These alternatives provide similar HRI capabilities while maintaining open-source accessibility for educational purposes.

## Key Concepts Learned

Students will learn:

1. How to configure realistic sensor simulation in Gazebo
2. How to integrate sensors into URDF models using xacro macros
3. How to access and process sensor data streams
4. How to validate that sensors are producing realistic data
5. How to troubleshoot sensor configurations
6. How to choose appropriate simulation environments for different use cases

## Validation

The sensor simulation can be validated by:

1. Confirming that all sensor topics are publishing data
2. Verifying that data ranges are within expected bounds
3. Checking that sensors respond appropriately to environment changes
4. Testing that noise models provide realistic data variation