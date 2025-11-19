# Neovio Autonomous Robot Car

An autonomous robot car system built for Raspberry Pi 5 with RP Lidar A1, implementing SLAM (Simultaneous Localization and Mapping) using ROS (Robot Operating System).

## Hardware Components

- **Raspberry Pi 5** running Ubuntu
- **RP Lidar A1** - 2D LiDAR sensor for mapping and obstacle detection
- **Raspberry Pi Camera Module v2** - Vision sensor (optional for future use)
- **L298N Motor Driver** - Dual H-bridge motor controller
- **4 DC Motors** - Differential drive configuration
- **4 Wheels** - Robot locomotion

## Software Requirements

### ROS Installation

This project requires ROS Noetic (for Ubuntu 20.04) or ROS Melodic (for Ubuntu 18.04). Since you're using Ubuntu on Raspberry Pi 5, install ROS Noetic:

```bash
# Add ROS Noetic repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update package list
sudo apt update

# Install ROS Noetic Desktop Full
sudo apt install ros-noetic-desktop-full -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install build tools
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
```

### Required ROS Packages

Install the following ROS packages:

```bash
sudo apt install ros-noetic-gmapping -y
sudo apt install ros-noetic-navigation -y
sudo apt install ros-noetic-move-base -y
sudo apt install ros-noetic-rviz -y
sudo apt install ros-noetic-tf -y
sudo apt install ros-noetic-geometry-msgs -y
sudo apt install ros-noetic-sensor-msgs -y
sudo apt install ros-noetic-nav-msgs -y
```

### Python Dependencies

```bash
sudo apt install python3-pip -y
pip3 install gpiozero
```

### USB Serial Permissions

Add your user to the dialout group to access serial devices:

```bash
sudo usermod -a -G dialout $USER
```

**Note:** You may need to log out and log back in for this to take effect.

### Verify Setup

Run the setup check script to verify your installation:

```bash
cd ~/Documents/neovio
./setup_check.sh
```

This will check:
- ROS installation
- Required ROS packages
- Python dependencies
- USB permissions
- LiDAR device detection
- Workspace structure

## Project Setup

### 1. Clone and Build the Workspace

```bash
# Navigate to your workspace (this directory)
cd ~/Documents/neovio

# Build the catkin workspace
catkin_make

# Source the workspace
source devel/setup.bash

# Add to bashrc for persistence
echo "source ~/Documents/neovio/devel/setup.bash" >> ~/.bashrc
```

### 2. Hardware Wiring Configuration

#### L298N Motor Driver to Raspberry Pi GPIO

The default GPIO pin configuration in the code is:

**Left Motor:**
- Enable Pin: GPIO 22
- Forward Pin: GPIO 17
- Backward Pin: GPIO 27

**Right Motor:**
- Enable Pin: GPIO 25
- Forward Pin: GPIO 23
- Backward Pin: GPIO 24

**Note:** You can modify these pins in the launch files if your wiring is different.

#### RP Lidar A1 Connection

- Connect RP Lidar A1 to Raspberry Pi via USB (typically `/dev/ttyUSB0`)
- Ensure the device has proper permissions (see USB Serial Permissions above)

### 3. Configure Robot Parameters

Edit the launch files to match your robot's physical parameters:

**File:** `src/neovio_autonomous_car/launch/slam.launch`

Adjust these parameters if needed:
- `wheel_base`: Distance between left and right wheels (default: 0.26 meters)
- `wheel_radius`: Radius of wheels (default: 0.065 meters)
- `max_speed`: Maximum motor speed (default: 0.8)

**File:** `src/neovio_autonomous_car/launch/slam.launch`

Check the RPLIDAR serial port:
- `serial_port`: Usually `/dev/ttyUSB0` or `/dev/ttyUSB1`
- `serial_baudrate`: 115200 for RP Lidar A1

To find your LiDAR device:
```bash
ls -l /dev/ttyUSB*
```

## Usage

### Starting the System

#### 1. Start ROS Master

In the first terminal:

```bash
cd ~/Documents/neovio
source devel/setup.bash
roscore
```

#### 2. Start SLAM Mapping

In a second terminal:

```bash
cd ~/Documents/neovio
source devel/setup.bash
roslaunch neovio_autonomous_car slam.launch
```

This will:
- Start the robot base controller (motor control + odometry)
- Start the RP Lidar A1 node
- Start SLAM (gmapping)
- Set up all necessary TF transforms

#### 3. Visualize in RViz (Optional)

In a third terminal:

```bash
cd ~/Documents/neovio
source devel/setup.bash
rosrun rviz rviz -d src/neovio_autonomous_car/rviz/slam.rviz
```

#### 4. Control the Robot

You can control the robot using:

**Teleoperation (Keyboard):**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**Or publish commands directly:**
```bash
# Move forward
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# Stop
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Saving the Map

Once you've mapped your environment, save it:

```bash
# In a new terminal
cd ~/Documents/neovio
source devel/setup.bash
rosrun map_server map_saver -f maps/my_map
```

This will create:
- `maps/my_map.pgm` - The map image
- `maps/my_map.yaml` - Map metadata

### Autonomous Navigation (After Mapping)

Once you have a saved map, you can use autonomous navigation:

#### 1. Start Navigation Stack

```bash
cd ~/Documents/neovio
source devel/setup.bash
roslaunch neovio_autonomous_car complete_system.launch
```

**Note:** You'll need to modify `complete_system.launch` to load your saved map instead of running SLAM.

#### 2. Set Initial Pose

In RViz, use the "2D Pose Estimate" tool to set the robot's initial position on the map.

#### 3. Set Navigation Goal

In RViz, use the "2D Nav Goal" tool to set a destination. The robot will autonomously navigate to that location.

## Project Structure

```
neovio/
├── README.md                          # This file
├── src/
│   ├── CMakeLists.txt                # Workspace CMakeLists
│   ├── neovio_autonomous_car/        # Main robot package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── scripts/
│   │   │   ├── robot_base_controller.py  # Motor control + odometry
│   │   │   ├── motor_control_node.py     # Legacy motor control
│   │   │   └── voice_command_node.py     # Voice commands (optional)
│   │   ├── launch/
│   │   │   ├── slam.launch              # SLAM mapping launch file
│   │   │   ├── navigation.launch        # Navigation stack
│   │   │   ├── complete_system.launch   # Full system
│   │   │   ├── gmapping.launch          # SLAM configuration
│   │   │   └── rplidar.launch           # LiDAR configuration
│   │   ├── config/
│   │   │   ├── base_local_planner_params.yaml
│   │   │   ├── costmap_common_params.yaml
│   │   │   ├── global_costmap_params.yaml
│   │   │   └── local_costmap_params.yaml
│   │   └── rviz/
│   │       ├── slam.rviz               # RViz config for SLAM
│   │       └── navigation.rviz         # RViz config for navigation
│   └── rplidar_ros/                   # RP Lidar ROS driver
└── devel/                              # Build output (after catkin_make)
└── build/                              # Build files (after catkin_make)
```

## Troubleshooting

### LiDAR Not Detected

1. Check USB connection:
   ```bash
   ls -l /dev/ttyUSB*
   ```

2. Check permissions:
   ```bash
   groups  # Should include 'dialout'
   ```

3. Try different serial port in launch file (`/dev/ttyUSB1`, `/dev/ttyACM0`, etc.)

### Motors Not Responding

1. Check GPIO pin configuration matches your wiring
2. Verify L298N power supply is connected
3. Check motor connections to L298N
4. Test motors directly (bypass ROS) to verify hardware

### SLAM Not Working

1. Ensure odometry is being published:
   ```bash
   rostopic echo /odom
   ```

2. Check LiDAR scan data:
   ```bash
   rostopic echo /scan
   ```

3. Verify TF tree:
   ```bash
   rosrun tf view_frames
   evince frames.pdf
   ```

4. Check for TF errors:
   ```bash
   rosrun tf tf_echo map odom
   ```

### Build Errors

1. Ensure all dependencies are installed
2. Clean and rebuild:
   ```bash
   cd ~/Documents/neovio
   rm -rf build devel
   catkin_make
   ```

## Configuration Tips

### Adjusting SLAM Parameters

Edit `src/neovio_autonomous_car/launch/gmapping.launch` to tune SLAM performance:
- `particles`: Number of particles (more = better but slower, default: 30)
- `maxUrange`: Maximum usable range of laser (default: 6.0)
- `linearUpdate`: Update map after moving this distance (default: 1.0)
- `angularUpdate`: Update map after rotating this angle (default: 0.5)

### Adjusting Navigation Parameters

Edit files in `src/neovio_autonomous_car/config/`:
- `base_local_planner_params.yaml`: Local planner behavior
- `costmap_common_params.yaml`: Obstacle detection settings
- `local_costmap_params.yaml`: Local costmap size and resolution
- `global_costmap_params.yaml`: Global costmap settings

## Next Steps

1. **Improve Odometry**: Integrate wheel encoders for more accurate odometry
2. **Camera Integration**: Add camera node for visual SLAM or object detection
3. **Voice Commands**: Implement voice control using the voice_command_node
4. **Map Server**: Set up persistent map loading for navigation
5. **Safety Features**: Add emergency stop and obstacle avoidance behaviors

## License

MIT License

## Support

For issues and questions, please check:
- ROS Wiki: http://wiki.ros.org/
- RP Lidar Documentation: https://github.com/robopeak/rplidar_ros
