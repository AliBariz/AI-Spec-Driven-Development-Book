# Running the Physical AI & Humanoid Robotics Project

This guide explains how to set up and run the Physical AI & Humanoid Robotics project on your local machine.

## Prerequisites

Before running the project, you need to install:

1. **ROS 2 Humble Hawksbill** (or similar version) - http://docs.ros.org/en/humble/Installation.html
2. **Gazebo Garden** - https://gazebosim.org/docs/garden/install
3. **Python 3.8+**
4. **Docusaurus prerequisites** (Node.js, npm)

## Installation Steps

### 1. Install ROS 2 Humble
Follow the official installation guide for your operating system:
- Ubuntu: http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- Windows: http://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html

### 2. Source ROS 2
```bash
# On Ubuntu
source /opt/ros/humble/setup.bash

# On Windows (in PowerShell)
[Environment]::SetEnvironmentVariable('AMENT_PYTHON_EXEC_LIBRARY', '1', [EnvironmentVariableTarget]::Process)
cmd /c 'echo off && set && path'
```

### 3. Clone the repository
```bash
git clone <repository-url>
cd humanoid-robotics-hackathon
```

### 4. Build the ROS workspace
```bash
cd ros_ws
colcon build --packages-select humanoid_description humanoid_control nav2_config vslam_pipeline voice_interface llm_planner cv_detection manipulation humanoid_vla
source install/setup.bash
```

## Running the Project

### Option 1: Basic Robot Simulation
```bash
# Terminal 1: Launch the robot in Gazebo
cd ros_ws
source install/setup.bash
ros2 launch humanoid_description gazebo.launch.py
```

```bash
# Terminal 2: Run the control agent
cd ros_ws
source install/setup.bash
ros2 run humanoid_control agent
```

### Option 2: Digital Twin with Sensors
```bash
# Terminal 1: Launch with full sensor simulation
cd ros_ws
source install/setup.bash
ros2 launch humanoid_description gazebo.launch.py
```

```bash
# Terminal 2: Test sensor data
cd ros_ws
source install/setup.bash
python3 src/humanoid_control/test/test_sensor_simulation.py
```

### Option 3: Complete VLA (Vision-Language-Action) System
```bash
# Terminal 1: Launch the complete VLA integration
cd ros_ws
source install/setup.bash
ros2 launch humanoid_vla vla_integration.launch.py
```

```bash
# Terminal 2: Send a test command
ros2 topic pub /voice/transcription std_msgs/String "data: 'move forward'"
```

### Option 4: Navigation System
```bash
# Terminal 1: Launch Nav2 for the humanoid robot
cd ros_ws
source install/setup.bash
ros2 launch nav2_config nav2.launch.py
```

```bash
# Terminal 2: Send a navigation goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position:
    x: 1.0
    y: 1.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

## Running Documentation Locally

To run the documentation website locally:

```bash
cd docs
npm install
npm start
```

The documentation will be available at http://localhost:3000

## Testing

Run the sensor simulation test to verify the system:
```bash
cd ros_ws
source install/setup.bash
python3 src/humanoid_control/test/test_sensor_simulation.py
```

## Troubleshooting

### Common Issues

1. **Package not found errors**: Make sure to source the ROS workspace:
   ```bash
   cd ros_ws
   source install/setup.bash
   ```

2. **Gazebo not launching**: Check that Gazebo is properly installed:
   ```bash
   gazebo --version
   ```

3. **Python import errors**: Install required dependencies:
   ```bash
   pip3 install opencv-python cv-bridge
   ```

4. **Permission errors**: Make sure your user has proper ROS 2 permissions

### Verifying Installation

To verify that all components are working:

1. Check available ROS packages:
   ```bash
   cd ros_ws
   source install/setup.bash
   ros2 pkg list | grep humanoid
   ```

2. Check available launch files:
   ```bash
   ros2 launch --list
   ```

## Development Workflow

For development, you can run individual components:

```bash
# Run just the voice interface
ros2 run voice_interface whisper_node

# Run just the object detection
ros2 run cv_detection object_detector

# Run just the LLM planner
ros2 run llm_planner planner

# Run just the manipulation system
ros2 run manipulation grasp_planner
```

## System Requirements

- **OS**: Ubuntu 22.04 LTS or Windows 10/11
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 10GB free space
- **GPU**: Optional but recommended for Isaac ROS components
- **Processor**: Multi-core processor recommended for simulation

## Next Steps

1. Follow the [Quickstart Guide](./docs/quickstart.md) for detailed instructions
2. Explore individual modules in the documentation
3. Try the example commands in the terminal
4. Experiment with different robot behaviors