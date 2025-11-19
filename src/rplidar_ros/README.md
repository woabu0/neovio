# RPLIDAR ROS Package

Custom ROS package for interfacing with RPLIDAR A1 sensors in the Neovio Autonomous Car project.

## Overview

This package provides a ROS node (`rplidarNode`) that interfaces with RPLIDAR A1 sensors and publishes laser scan data to the `/scan` topic in the standard `sensor_msgs/LaserScan` format.

## Features

- Serial communication with RPLIDAR A1
- Publishes laser scan data to `/scan` topic
- Motor control services (`start_motor`, `stop_motor`)
- Configurable parameters (serial port, baudrate, frame_id, etc.)
- Angle compensation support

## Requirements

- ROS Noetic (or compatible ROS version)
- SLAMTEC RPLIDAR SDK (required for hardware communication)
- RPLIDAR A1 sensor connected via USB serial port

## Setup

### 1. Install SLAMTEC RPLIDAR SDK

The SDK is required for hardware communication. You can obtain it from:
- Official SLAMTEC website
- Or include it as a subdirectory in this package

### 2. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Configure Serial Port Permissions

```bash
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

## Usage

### Basic Launch

```bash
roslaunch rplidar_ros rplidar_a1.launch
```

### With Custom Parameters

```bash
roslaunch rplidar_ros rplidar_a1.launch serial_port:=/dev/ttyUSB1
```

### Test Client

```bash
rosrun rplidar_ros rplidarNodeClient
```

## Parameters

- `serial_port` (string, default: "/dev/ttyUSB0"): Serial port device path
- `serial_baudrate` (int, default: 115200): Serial communication baudrate
- `frame_id` (string, default: "laser"): TF frame ID for the laser
- `inverted` (bool, default: false): Invert the scan data
- `angle_compensate` (bool, default: true): Enable angle compensation

## Topics

- `/scan` (sensor_msgs/LaserScan): Published laser scan data

## Services

- `/start_motor` (std_srvs/Empty): Start the RPLIDAR motor
- `/stop_motor` (std_srvs/Empty): Stop the RPLIDAR motor

## Notes

This is a custom implementation created for the Neovio Autonomous Car project. The code is original and does not contain any cloned repositories.

