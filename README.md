# Autonomous Navigation of a Tugbot in a Warehouse

## Project Overview

This repository provides the ROS 2 nodes to autonomously control a tugbot in a warehouse in a virtual environment. 
The ROS 2 Driver node that was designed in C++ utilizes the robot’s yaw and Cartesian position to continuously adjust its linear and angular velocity. 
Moreover, there's a Velodyne LiDAR mounted on the tugbot, but the Driver node considers only the sensor's data in the X and Y dimensions. 

## Software Setup

Install the software. 

- Linux Distribution: Ubuntu 22.04 Jazzy.
- ROS 2 Humble
- Gazebo Harmonic 8.10.0
- Python 3.10
- C++

The documentation below contains instructions on how to install ROS 2 Humble. Always build and source the package in the ROS 2 workspace before running the nodes. 

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

## Simulation Testing

**First Terminal** 
  - Execute the command below to convert the velocity topic from Gazebo to ROS 2 format. This is the topic over which the Driver node will publish the linear and angular velocities of the tugbot.
  - ros2 run ros_gz_bridge parameter_bridge /model/tugbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist 

**Second Terminal**
  - Execute the script below to run a ROS 2 node that will extract the current Cartesian positions of the tugbot.  
  - python3 position_bridge.py

**Third Terminal**
  - Execute the script below to run a ROS 2 node that will extract the current orientation of the tugbot.  
  - python3 orientation_bridge.py

**Fourth Terminal**
  - Execute the script below to run a ROS 2 node that will capture the distance measurements of the Velodyne LiDAR sensor.  
  - python3 lidar_bridged.py

**Fifth Terminal**
  - Execute the script below to run the driver node.
  - ros2 run drive controller

**Sixth Terminal (Optional)**
  - Run the command below that illustrates the communication between the ROS 2 nodes.
  - rqt_graph

The tugbot's behavior is shown in the video link below after running the ROS 2 nodes.

https://www.youtube.com/watch?v=u5uOXOIH8SA

## Author
  
- **Name**: Ahsan Akhlaque
- **Email**: aakhl097@uottawa.ca

