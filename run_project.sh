#!/bin/bash

# Physical AI & Humanoid Robotics - Quick Start Script

echo "==========================================="
echo "Physical AI & Humanoid Robotics Project"
echo "==========================================="

# Check if ROS 2 is installed
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 is not installed or sourced."
    echo "Please install ROS 2 Humble and source the setup file:"
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS 2 Distribution: $ROS_DISTRO"
echo "Current directory: $(pwd)"
echo ""

# Check if we're in the right directory
if [ ! -d "ros_ws" ]; then
    echo "Error: ros_ws directory not found."
    echo "Please run this script from the project root directory."
    exit 1
fi

# Function to display menu
show_menu() {
    echo "Select an option:"
    echo "1. Build the project"
    echo "2. Launch basic robot simulation"
    echo "3. Launch with sensor simulation"
    echo "4. Launch complete VLA system"
    echo "5. Launch navigation system"
    echo "6. Run sensor test"
    echo "7. Run documentation locally"
    echo "8. Help"
    echo "0. Exit"
    echo -n "Enter your choice [0-8]: "
}

# Function to build the project
build_project() {
    echo "Building the ROS workspace..."
    cd ros_ws
    colcon build --packages-select humanoid_description humanoid_control nav2_config vslam_pipeline voice_interface llm_planner cv_detection manipulation humanoid_vla
    source install/setup.bash
    cd ..
    echo "Build completed!"
}

# Function to launch basic simulation
launch_basic() {
    echo "Launching basic robot simulation..."
    echo "Run this in a separate terminal:"
    echo "  cd ros_ws && source install/setup.bash && ros2 launch humanoid_description gazebo.launch.py"
    echo ""
    echo "Then in another terminal:"
    echo "  cd ros_ws && source install/setup.bash && ros2 run humanoid_control agent"
}

# Function to launch with sensors
launch_sensors() {
    echo "Launching robot with sensor simulation..."
    echo "Run this in a separate terminal:"
    echo "  cd ros_ws && source install/setup.bash && ros2 launch humanoid_description gazebo.launch.py"
    echo ""
    echo "Then test sensors in another terminal:"
    echo "  cd ros_ws && source install/setup.bash && python3 src/humanoid_control/test/test_sensor_simulation.py"
}

# Function to launch VLA system
launch_vla() {
    echo "Launching complete VLA system..."
    echo "Run this in a separate terminal:"
    echo "  cd ros_ws && source install/setup.bash && ros2 launch humanoid_vla vla_integration.launch.py"
    echo ""
    echo "Then send a test command in another terminal:"
    echo "  ros2 topic pub /voice/transcription std_msgs/String \"data: 'move forward'\""
}

# Function to launch navigation
launch_nav() {
    echo "Launching navigation system..."
    echo "Run this in a separate terminal:"
    echo "  cd ros_ws && source install/setup.bash && ros2 launch nav2_config nav2.launch.py"
}

# Function to run sensor test
run_sensor_test() {
    echo "Running sensor test..."
    echo "Run this command:"
    echo "  cd ros_ws && source install/setup.bash && python3 src/humanoid_control/test/test_sensor_simulation.py"
}

# Function to run documentation
run_docs() {
    echo "To run documentation locally:"
    echo "  cd docs && npm install && npm start"
    echo ""
    echo "Documentation will be available at http://localhost:3000"
}

# Function to show help
show_help() {
    echo "This project requires ROS 2 Humble, Gazebo, and other dependencies."
    echo ""
    echo "Before running any option, make sure to:"
    echo "1. Install ROS 2 Humble"
    echo "2. Install Gazebo Garden"
    echo "3. Install Python 3.8+"
    echo "4. Clone this repository"
    echo ""
    echo "For complete instructions, see RUNNING_THE_PROJECT.md"
    echo ""
}

# Main loop
while true; do
    show_menu
    read choice

    case $choice in
        1) build_project ;;
        2) launch_basic ;;
        3) launch_sensors ;;
        4) launch_vla ;;
        5) launch_nav ;;
        6) run_sensor_test ;;
        7) run_docs ;;
        8) show_help ;;
        0) echo "Exiting..."; exit 0 ;;
        *) echo "Invalid option. Please select 0-8." ;;
    esac

    echo ""
    echo -n "Press Enter to continue..."
    read
    echo ""
done