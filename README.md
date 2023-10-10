# Ball Tracking with ROS 2

This repository contains files to track a ball based on its color using OpenCV functions and send its 3D coordinates using ROS 2.

## Overview

The system utilizes ROS 2's publisher to transfer ball tracking details, allowing for real-time monitoring and analysis.

## Prerequisites

Before running the provided code, make sure you have the following prerequisites set up:

1. **ROS 2 Installation**:
   - Install ROS 2 (Humble Hawksbill or newer versions) by following the official installation instructions for your specific operating system: [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html).

2. **Create a ROS 2 Workspace**:
   - Create a workspace for your ROS 2 packages using the following commands:
     ```bash
     mkdir -p ~/ros2_ws/src
     cd ~/ros2_ws
     colcon build --symlink-install
     ```

3. **ROS 2 Python Package**:
   - Create a new ROS 2 package (e.g., `ball_tracker`) within the `src` directory of your workspace:
     ```bash
     cd ~/ros2_ws/src
     ros2 pkg create ball_tracker
     ```

4. **Dependencies Installation**:
   - Install the necessary ROS 2 dependencies for the `ball_tracker` package:
     ```bash
     rosdep install -i --from-path ~/ros2_ws/src --rosdistro humble -y
     ```

5. **Build the ROS 2 Workspace**:
   - Build the ROS 2 workspace to compile the package and its dependencies:
     ```bash
     cd ~/ros2_ws
     colcon build --symlink-install
     ```

6. **Camera Publisher Node**:
   - Ensure you have a separate ROS 2 node or package publishing images on the specified image topic (e.g., `/camera/image_raw`), which is subscribed to by the `BallTrackerNode` in the provided code.

7. **Camera Calibration Data**:
   - Obtain camera calibration data (camera matrix `mtx` and distortion coefficients `dist`) using the camera calibration code we have . Save this data in an appropriate file format (e.g., `.npz`) and provide the correct file path in the provided code.

Once you have the ROS 2 environment set up, the required package created, and the camera publishing images on the specified topic, you can run the provided code within the `ball_tracker` package using ROS 2 tools.