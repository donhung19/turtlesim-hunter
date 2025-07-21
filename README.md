# turtlesim-hunter

Developed a ROS2 simulation where a turtlebot autonomously tracks and catches the nearest spawned turtles using C++ nodes, A* pathfinding, real-time feedback, and ROS2 services. 


## Features

- Automatically spawns turtles at random positions.
- turtle1 chases and kills the turtle.
- Uses ROS2 publisher/subscriber, service/client, and timer.

## Requirements

- ROS 2 Jazzy (or your version)
- turtlesim package

## Usage

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build
colcon build
source install/setup.bash

# Launch
ros2 launch my_turtle my_turtle.launch.xml

## Video Demo
https://youtu.be/IGEXs73J0Mo
