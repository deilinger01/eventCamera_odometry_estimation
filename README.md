# README for Odometry Estimation Package
<span style="color:red">Important only tested for DAVIS346. </span>


## Overview
This package is designed to facilitate odometry estimation using event-based vision and optical flow techniques. 
The package includes several ROS launch files to streamline the initialization and operation of the system. 
Additionally, the `event_msg` folder is a critical component that must be built for the system to function properly.

## Folder Structure
📁 **odometry_estimation** \
├── 📁 **launch** \
│   ├── 📄 **odometry_estimation.launch**  — Main launch file to run all required nodes \
│   ├── 📄 **events_generation.launch**    — Launch file for event generation node \
│   └── 📄 **farneback.launch**            — Launch file for optical flow using Farneback algorithm \
│ \
├── 📁 **velocity_estimation** \
│   ├── 📄 **events_generation.py**        — Python script for event generation \
│   └── 📄 **Farneback.py**                — Python script for optical flow using Farneback algorithm \
│ \
├── 📁 **camera_holder** \
│   └── 📄 **Camera_Holder.stl**           — CAD model of the camera mount \
│ \
├── 📄 **CMakeLists.txt**                  — Build configuration file for ROS \
├── 📄 **package.xml**                     — Package metadata file for ROS \
├── 📄 **DAVIS-00826001.xml**              — Camera calibration file \
└── 📄 **README.md**                       — Project README file \
 \
 \
📁 **event_msg** \
├── 📁 **msg** \
│   └── 📄 **Events.msg**                  — Custom message definition for event data \
│ \
├── 📄 **CMakeLists.txt**                 — Build configuration file for ROS \
└── 📄 **package.xml**                    — Package metadata file for ROS 


## Prerequisites
Ensure that you have the following installed on your system:
- ROS (Robot Operating System) - Compatible with ROS1 (should be also compatible with ROS 2, but not tested)
- Python

## Installation
1. **Clone the Repository**
    git clone <TODO> 
    cd <TODO>

2. **Build the Workspace**
    with `catkin build` command
    Make sure to build the `event_msg` folder as it contains the custom message definitions required by the nodes.

3. **Source the Workspace**
    source devel/setup.bash  For ROS1

## Launch Files

### 1. **odometry_estimation.launch**
This is the main launch file that runs both the `events_generation` and `farneback` nodes simultaneously. 
It is intended to provide a complete odometry estimation pipeline. --> not tested yet

**Usage:**
roslaunch odometry_estimation odometry_estimation.launch

### 2. **events_generation.launch**
This launch file initializes only the event generation node. 
This node is responsible for generating event data that will be processed by other components in the system.

**Usage:**
roslaunch odometry_estimation events_generation.launch

### 3. **farneback.launch**
This launch file initializes only the optical flow calculation using the Farneback algorithm. 
It can be used independently to visualize or debug the optical flow data.

**Usage:**
roslaunch odometry_estimation farneback.launch

## Important Notes
- **event_msg**: This folder contains custom message definitions required for the event data. 
  It is crucial that you build this folder to avoid errors related to missing message types.
- **Build Step**: Make sure to run `catkin_make` or `colcon build` after any changes to the message files in `event_msg`.

## Troubleshooting
- **Missing Message Errors**: If you encounter issues related to missing message types, ensure that you have properly built the `event_msg` folder.
- **Node Not Found**: If any nodes are not found, ensure that the corresponding `.py` or executable files have the correct permissions (`chmod +x <filename>`).
- **No compatible device discovered**: Camera isn't connected. 