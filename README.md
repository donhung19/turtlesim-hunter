# turtlesim-hunter

A ROS2 package that spawns turtles in `turtlesim`, controls `turtle1` to approach and "kill" the one, and demonstrates asynchronous service/client communication, velocity control, and timer-based logic.

## 📦 Features

- Automatically spawns turtles at random positions.
- turtle1 chases and kills the turtle.
- Uses ROS2 publisher/subscriber, service/client, and timer.

## 🧰 Requirements

- ROS 2 Jazzy (or your version)
- turtlesim package

## 🚀 Usage

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build
colcon build
source install/setup.bash

# Launch
ros2 launch my_turtle my_turtle.launch.xml
