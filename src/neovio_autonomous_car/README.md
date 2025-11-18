# Neovio Autonomous Car

An autonomous car project for Raspberry Pi 5 using ROS Noetic, including SLAM, navigation, obstacle avoidance, and voice command control.

## Hardware Requirements

- Raspberry Pi 5
- Raspberry Pi Camera Module v2 (optional for advanced features)
- RPLIDAR A1 (connected to /dev/ttyUSB0)
- 4 DC motors with wheels (differential drive configuration)
- L298N motor driver (2 units recommended for 4 motors)
- Microphone for voice commands
- Ubuntu 20.04 installed on Raspberry Pi

## Software Setup

### 1. Install ROS Noetic on Raspberry Pi

```bash
sudo apt update
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-base
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Required ROS Packages

```bash
sudo apt install ros-noetic-gmapping ros-noetic-navigation ros-noetic-amcl ros-noetic-map-server ros-noetic-rviz ros-noetic-teleop-tools python3-gpiozero
```

### 3. Install Additional Python Packages

```bash
pip3 install speechrecognition pyaudio
```

### 4. Set Up Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
# Copy the neovio_autonomous_car package to this directory
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Hardware Wiring

### L298N Motor Driver

For 4 motors (2 per side), use 2 L298N drivers:

**Left Side (L298N #1):**
- IN1: GPIO 17, IN2: GPIO 18, ENA: GPIO 22
- Connect to Left Front and Left Rear motors

**Right Side (L298N #2):**
- IN1: GPIO 23, IN2: GPIO 24, ENA: GPIO 25
- Connect to Right Front and Right Rear motors

Adjust GPIO pins in `scripts/motor_control_node.py` if using different pins.

## Usage

### 1. Make Scripts Executable

```bash
cd ~/catkin_ws/src/neovio_autonomous_car
chmod +x scripts/*.py
```

### 2. Build Map (First Time Setup)

```bash
# Start LIDAR
roslaunch rplidar_ros rplidar.launch

# In another terminal
roslaunch neovio_autonomous_car gmapping.launch

# In another terminal, teleop to drive around
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Drive the robot around to build the map
# When done, save the map
rosrun map_server map_saver -f ~/my_map
```

### 3. Run Autonomous Navigation

```bash
roslaunch neovio_autonomous_car navigation.launch
```

In separate terminals:

```bash
# Voice commands
rosrun neovio_autonomous_car voice_command_node.py

# Optional: RViz for visualization
rosrun rviz rviz
```

### 4. Voice Commands

- "Go to fridge" - Navigate to predefined location
- "Move forward" - Manual forward movement
- "Turn left/right" - Manual turns
- "Stop" - Stop all movement

Known locations: fridge, kitchen, living room (customize in `voice_command_node.py`)

### 5. Tuning

- Adjust locations in `voice_command_node.py`
- Modify robot footprint and speeds in config files
- Tune gmapping parameters for better mapping

## Architecture

- **motor_control_node.py**: Controls motors via GPIO based on /cmd_vel topic
- **voice_command_node.py**: Processes speech input and sends navigation goals
- **RPLIDAR driver**: Provides laser scan data for SLAM and obstacle avoidance
- **gmapping**: Performs online SLAM to build and update map
- **move_base**: Navigation stack for path planning and control

## Troubleshooting

- Ensure RPLIDAR is connected to /dev/ttyUSB0
- Check GPIO permissions: `sudo usermod -a -G gpio $USER`
- Install PortAudio for speech recognition: `sudo apt install portaudio19-dev`
- If motors don't move, check L298N connections and power supply

## Future Improvements

- Add camera integration for object detection
- Implement more sophisticated voice commands
- Add odometry for better localization
- Integrate with mapping for dynamic environment updates
