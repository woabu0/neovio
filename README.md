# Autonomous Robot Car with SLAM

Autonomous robot car implementation using Raspberry Pi 5, RPLidar A1, L298N motor controller, and Python-based SLAM.

## Features

- **Fixed Buffer Issues**: Improved LiDAR data handling with threading and efficient buffer management
- **SLAM Implementation**: Occupancy grid mapping using inverse sensor model
- **Autonomous Navigation**: Obstacle avoidance with real-time map building
- **Real-time Visualization**: Live map visualization with robot path tracking

## Hardware Requirements

- Raspberry Pi 5
- RPLidar A1
- L298N Motor Controller
- 4 DC Motors with 4 Wheels
- USB to Serial adapter (for LiDAR)

## Software Dependencies

Install required packages:

```bash
pip install -r requirements.txt
```

Or install individually:
```bash
pip install rplidar-roboticia gpiozero numpy scipy matplotlib pyserial
```

## Project Structure

```
NewCar499/
├── lidar_handler.py      # Improved LiDAR handler with buffer management
├── slam_mapper.py        # SLAM occupancy grid implementation
├── autonomous_slam.py    # Main autonomous navigation with SLAM
├── motor_control.py      # Motor driver interface
├── lidar_test.py         # Test script for LiDAR (improved)
├── requirements.txt      # Python dependencies
└── README.md            # This file
```

## Usage

### 1. Test LiDAR (Fixed Buffer Issue)

Test the improved LiDAR handler:

```bash
python3 lidar_test.py
```

This script uses the new `LidarHandler` class which:
- Uses threading to prevent buffer overflow
- Manages data efficiently with queue-based processing
- Handles up to 3600 measurements per scan
- Provides thread-safe access to scan data

### 2. Run Autonomous SLAM

Run the complete autonomous system with SLAM:

```bash
python3 autonomous_slam.py
```

The robot will:
- Initialize LiDAR and motors
- Build a map as it moves
- Avoid obstacles autonomously
- Display real-time map visualization
- Save map to `slam_map.npy` on exit

### 3. Configuration

Edit `autonomous_slam.py` to customize:

```python
config = {
    'lidar_port': '/dev/ttyUSB0',      # LiDAR serial port
    'lidar_pwm': 500,                   # LiDAR motor speed (300-600)
    'left_pins': (17, 22, 25),          # Left motor GPIO pins
    'right_pins': (23, 24, 5),          # Right motor GPIO pins
    'map_size': 800,                    # Map size in pixels
    'map_resolution': 0.05,             # Meters per pixel (5cm)
    'obstacle_threshold': 400,          # Obstacle distance in mm
    'forward_speed': 0.5,               # Forward speed (0-1)
    'turn_speed': 0.5,                  # Turn speed (0-1)
    'show_map': True                    # Show live map visualization
}
```

## Key Improvements

### Buffer Issue Fix

The original code had max buffer overflow issues. The new `LidarHandler` class fixes this by:

1. **Threading**: Separate thread for LiDAR data collection
2. **Queue Management**: Bounded queue prevents memory overflow
3. **Efficient Processing**: Processes scans without blocking
4. **Increased Buffer**: Supports up to 3600 measurements per scan
5. **Thread-Safe Access**: Safe concurrent access to scan data

### SLAM Implementation

The `SLAMMapper` class implements:

1. **Occupancy Grid Mapping**: Log-odds representation for numerical stability
2. **Inverse Sensor Model**: Marks occupied and free cells along LiDAR rays
3. **Pose Tracking**: Tracks robot position and orientation
4. **Path Recording**: Records robot trajectory for visualization
5. **Map Persistence**: Save/load maps to/from files

## Troubleshooting

### LiDAR Not Detected

1. Check serial port: `ls -l /dev/ttyUSB*`
2. Ensure permissions: `sudo usermod -a -G dialout $USER` (logout/login)
3. Verify connection: `dmesg | tail` after plugging in

### Buffer Overflow Still Occurs

1. Increase `max_buffer` in `LidarHandler` initialization
2. Reduce `lidar_pwm` speed (try 400-450)
3. Check USB connection quality

### Motors Not Working

1. Verify GPIO pins match your wiring
2. Check L298N power supply (12V recommended)
3. Ensure proper ground connections

### Map Not Building

1. Ensure LiDAR is scanning: check `lidar_test.py`
2. Verify robot is moving (check motor connections)
3. Adjust `map_resolution` if map is too small/large

## Advanced Usage

### Load Saved Map

```python
from slam_mapper import SLAMMapper

slam = SLAMMapper()
slam.load_map('slam_map.npy')
slam.visualize()
```

### Custom Obstacle Avoidance

Modify the `obstacle_avoidance()` method in `autonomous_slam.py` to implement custom behaviors.

### Add Wheel Encoders

For better odometry, integrate wheel encoders and update the `estimate_odometry()` method.

## License

This project is part of the Neovio Autonomous Car project.
