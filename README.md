# slam_center: message exchange center for ROS2 humble

This ROS2 humble package provides a center connecting [camera node from RPI OS]() and [neural network-based SLAM algorithm](), which exchange messages via **ROS2 topics** and **ZMQ sockets**.

## Table of Contents

- [Requirements](#requirements)
- [About](#about)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)

---

## Requirements

This package is tested on the following environment configuration:

- Ubuntu22.04
- [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- system python 3.10 (without virtual env)

---

## About

The following diagram describes the function and relation of this `slam_center` package and other parts of the project [real-time VSLAM based on Raspberry Pi 5 and 3D scene reconstruction model via ROS2 humble]():

<img src="slam_center_diagram.svg" width="800"/>

1. the `image topic` receives compressed image from [ROS camera node of ROS2 humble in docker container on Raspberry Pi OS](), and sends them to image zmq socket;

2. the `image zmq socket` sends the compressed image to [neural network-based SLAM model]();

3. the `pcd zmq sockets` and `pose zmq sockets` receive the result 3D point cloud and camera poses from SLAM model, and send them to corresponding topic;

4. the `pcd topic` and `pose topic` receive corresponding results from zmq sockets, send them to `Rviz` for real-time visualization;

---

## Features

- Real-time message exchange
- Message exchange between environments with different python versions
- Exchange compressed image, 3D point cloud, and camera poses
- Visualize result with `Rviz` in real time

---

## Installation

Make sure [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) is installed successfully before installing this package.

```bash
# create and enter a ROS2 workspace
mkdirs -p ros_rtp_ws/src/ # rtp: real-time perception
cd ros_rtp_ws/
rosdep install -i --from-path src --rosdistro humble -y # check if all necessary dependencies are ready

# Clone the repository
cd src/
git clone git@github.com:MyLovelyAxe/slam_center.git

# Build package
cd .. # return to workspace
colcon build --packages-select slam_center --symlink-install
```

## Usage

Download a rosbag containing continuous recorded compressed images [here](). Open a terminal, play the rosbag to publish the images:

```bash
cd ros_rtp_ws/
source install/setup.bash # source ROS2 underlay and overlay
# 1. play once
ros2 bag play rosbag2_2025_05_26-23_56_14 
# 2. play forever
# while true; do ros2 bag play rosbag2_2025_05_26-23_56_14; done
```

Open **another** terminal, run node `/send_comp_img` which receives compressed images from rosbag and sends to image socket:

```bash
cd ros_rtp_ws/
source install/setup.bash
ros2 run slam_center send_comp_img
```

Open **another** terminal, refer to [here]() to test if the compressed image can be received by zmq socket under a different python version (i.e. python 3.11) for SLAM model.