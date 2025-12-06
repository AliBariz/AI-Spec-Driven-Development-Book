# Quickstart Guide

Get started with the Physical AI & Humanoid Robotics project in minutes.

## Prerequisites

- ROS 2 Humble Hawksbill (or similar)
- Gazebo simulation environment
- Python 3.8+
- Docusaurus prerequisites (Node.js, npm)

## Setup

### 1. Clone the repository

```bash
git clone <repository-url>
cd humanoid-robotics-hackathon
```

### 2. Install ROS 2 dependencies

```bash
# Install ROS 2 Humble (Ubuntu/Debian)
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### 3. Build the ROS workspace

```bash
cd ros_ws
colcon build --packages-select humanoid_description humanoid_control nav2_config vslam_pipeline voice_interface llm_planner cv_detection manipulation humanoid_vla
source install/setup.bash
```

## Running the Modules

### Module 1: ROS 2 Control Pipeline

Launch the basic control pipeline:

```bash
# Launch the robot in Gazebo
ros2 launch humanoid_description gazebo.launch.py

# In another terminal, run the control agent
ros2 run humanoid_control agent
```

### Module 2: Digital Twin Environment

Launch the robot with full sensor simulation:

```bash
# Launch the robot in Gazebo with all sensors
ros2 launch humanoid_description gazebo.launch.py

# Test sensor data
python3 ros_ws/src/humanoid_control/test/test_sensor_simulation.py
```

### Module 3: AI-Robot Brain

Launch the navigation system:

```bash
# Launch Nav2 for the humanoid robot
ros2 launch nav2_config nav2.launch.py

# Launch VSLAM pipeline
ros2 run vslam_pipeline vslam_node
```

### Module 4: Vision-Language-Action Capstone

Launch the complete VLA system:

```bash
# Launch the full VLA integration
ros2 launch humanoid_vla vla_integration.launch.py

# Send a test command
ros2 topic pub /voice/transcription std_msgs/String "data: 'move forward'"
```

## Documentation

Build and serve the documentation locally:

```bash
cd docs
npm install
npm start
```

## Testing

Run the sensor simulation test:

```bash
python3 ros_ws/src/humanoid_control/test/test_sensor_simulation.py
```

## Troubleshooting

### Common Issues

1. **Package not found**: Make sure to source the ROS workspace:
   ```bash
   source ros_ws/install/setup.bash
   ```

2. **Gazebo not launching**: Check that Gazebo is properly installed:
   ```bash
   gazebo --version
   ```

3. **Python import errors**: Make sure all dependencies are installed:
   ```bash
   pip3 install opencv-python cv-bridge
   ```

## Next Steps

1. Follow the [Module 1: ROS 2 Control Pipeline](/docs/module1/implementation) guide
2. Explore the [Module 2: Digital Twin Environment](/docs/module2/implementation)
3. Implement the [Module 3: AI-Robot Brain](/docs/module3/implementation)
4. Build the [Module 4: Vision-Language-Action Capstone](/docs/capstone/implementation)

## Support

For questions and support, refer to the documentation or create an issue in the repository.