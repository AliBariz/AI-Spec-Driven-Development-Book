@echo off
REM Physical AI & Humanoid Robotics - Quick Start Batch File

echo ===========================================
echo Physical AI & Humanoid Robotics Project
echo ===========================================

REM Check if ROS 2 is installed
if "%ROS_DISTRO%"=="" (
    echo Error: ROS 2 is not installed or sourced.
    echo Please install ROS 2 Humble and source the setup file:
    echo   Call the ROS 2 setup script appropriate for your installation
    pause
    exit /b 1
)

echo ROS 2 Distribution: %ROS_DISTRO%
echo Current directory: %cd%
echo.

REM Check if we're in the right directory
if not exist "ros_ws" (
    echo Error: ros_ws directory not found.
    echo Please run this script from the project root directory.
    pause
    exit /b 1
)

:menu
echo Select an option:
echo 1. Build the project
echo 2. Launch basic robot simulation
echo 3. Launch with sensor simulation
echo 4. Launch complete VLA system
echo 5. Launch navigation system
echo 6. Run sensor test
echo 7. Run documentation locally
echo 8. Help
echo 0. Exit
set /p choice="Enter your choice [0-8]: "

if "%choice%"=="1" goto build
if "%choice%"=="2" goto basic
if "%choice%"=="3" goto sensors
if "%choice%"=="4" goto vla
if "%choice%"=="5" goto nav
if "%choice%"=="6" goto test
if "%choice%"=="7" goto docs
if "%choice%"=="8" goto help
if "%choice%"=="0" goto exit
echo Invalid option. Please select 0-8.
pause
goto menu

:build
echo Building the ROS workspace...
echo cd ros_ws
echo colcon build --packages-select humanoid_description humanoid_control nav2_config vslam_pipeline voice_interface llm_planner cv_detection manipulation humanoid_vla
echo call install\setup.bat
echo cd ..
echo Build completed!
pause
goto menu

:basic
echo Launching basic robot simulation...
echo Run this in a separate command prompt:
echo   cd ros_ws && call install\setup.bat && ros2 launch humanoid_description gazebo.launch.py
echo.
echo Then in another command prompt:
echo   cd ros_ws && call install\setup.bat && ros2 run humanoid_control agent
pause
goto menu

:sensors
echo Launching robot with sensor simulation...
echo Run this in a separate command prompt:
echo   cd ros_ws && call install\setup.bat && ros2 launch humanoid_description gazebo.launch.py
echo.
echo Then test sensors in another command prompt:
echo   cd ros_ws && call install\setup.bat && python src\humanoid_control\test\test_sensor_simulation.py
pause
goto menu

:vl
echo Launching complete VLA system...
echo Run this in a separate command prompt:
echo   cd ros_ws && call install\setup.bat && ros2 launch humanoid_vla vla_integration.launch.py
echo.
echo Then send a test command in another command prompt:
echo   ros2 topic pub /voice/transcription std_msgs/String "data: 'move forward'"
pause
goto menu

:nav
echo Launching navigation system...
echo Run this in a separate command prompt:
echo   cd ros_ws && call install\setup.bat && ros2 launch nav2_config nav2.launch.py
pause
goto menu

:test
echo Running sensor test...
echo Run this command:
echo   cd ros_ws && call install\setup.bat && python src\humanoid_control\test\test_sensor_simulation.py
pause
goto menu

:docs
echo To run documentation locally:
echo   cd docs && npm install && npm start
echo.
echo Documentation will be available at http://localhost:3000
pause
goto menu

:help
echo This project requires ROS 2 Humble, Gazebo, and other dependencies.
echo.
echo Before running any option, make sure to:
echo 1. Install ROS 2 Humble for Windows
echo 2. Install Gazebo Garden
echo 3. Install Python 3.8+
echo 4. Clone this repository
echo.
echo For complete instructions, see RUNNING_THE_PROJECT.md
echo.
pause
goto menu

:exit
echo Exiting...
pause