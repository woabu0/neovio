#!/usr/bin/env python3
"""
Autonomous Robot with SLAM
Integrates LiDAR, motor control, and SLAM for autonomous navigation
"""

import time
import signal
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
import sys

from lidar_handler import LidarHandler
from motor_control import MotorDriver
from slam_mapper import SLAMMapper


class AutonomousSlamRobot:
    """Main autonomous robot class with SLAM"""
    
    def __init__(self, config=None):
        """
        Initialize autonomous robot
        
        Config dict can contain:
            - lidar_port: Serial port for LiDAR (default: '/dev/ttyUSB0')
            - lidar_pwm: PWM speed for LiDAR (default: 500)
            - left_pins: Left motor pins (default: (17, 22, 25))
            - right_pins: Right motor pins (default: (23, 24, 5))
            - map_size: SLAM map size (default: 800)
            - map_resolution: Map resolution in m/pixel (default: 0.05)
            - obstacle_threshold: Distance threshold in mm (default: 400)
            - forward_speed: Forward speed 0-1 (default: 0.5)
            - turn_speed: Turn speed 0-1 (default: 0.5)
        """
        if config is None:
            config = {}
        
        # Configuration
        self.lidar_port = config.get('lidar_port', '/dev/ttyUSB0')
        self.lidar_pwm = config.get('lidar_pwm', 500)
        self.left_pins = config.get('left_pins', (17, 22, 25))
        self.right_pins = config.get('right_pins', (23, 24, 5))
        self.map_size = config.get('map_size', 800)
        self.map_resolution = config.get('map_resolution', 0.05)
        self.obstacle_threshold = config.get('obstacle_threshold', 400)
        self.forward_speed = config.get('forward_speed', 0.5)
        self.turn_speed = config.get('turn_speed', 0.5)
        self.show_map = config.get('show_map', True)
        
        # Components
        self.lidar = None
        self.motors = None
        self.slam = None
        
        # State
        self.running = False
        self.last_update_time = time.time()
        self.last_scan_time = time.time()
        self.scan_update_rate = 0.1  # Update SLAM every 100ms
        
        # Odometry (simple estimation based on motor commands)
        self.current_speed = 0.0
        self.current_angular_velocity = 0.0
        self.last_pose_update = time.time()
        
        # Visualization
        self.viz_thread = None
        self.fig = None
        self.ax = None
        
    def initialize(self):
        """Initialize all components"""
        print("Initializing autonomous robot...")
        
        # Initialize motors
        print("Initializing motors...")
        self.motors = MotorDriver(self.left_pins, self.right_pins)
        time.sleep(0.5)
        
        # Initialize SLAM
        print("Initializing SLAM mapper...")
        self.slam = SLAMMapper(
            map_size=self.map_size,
            resolution=self.map_resolution,
            max_range=4000
        )
        
        # Initialize LiDAR
        print("Initializing LiDAR...")
        self.lidar = LidarHandler(
            port=self.lidar_port,
            pwm_speed=self.lidar_pwm,
            max_buffer=3600
        )
        self.lidar.start()
        time.sleep(2)  # Wait for LiDAR to stabilize
        
        print("Robot initialized successfully!")
    
    def estimate_odometry(self, dt):
        """
        Estimate odometry from motor commands
        
        This is a simple estimation. For better accuracy, use wheel encoders.
        """
        # Simple model: assume constant speed during dt
        # Forward speed: ~0.1 m/s at speed 0.5
        # Angular speed: ~0.5 rad/s when turning
        
        forward_velocity = self.current_speed * 0.2  # m/s
        angular_velocity = self.current_angular_velocity * 1.0  # rad/s
        
        # Update pose
        dx = forward_velocity * dt * np.cos(self.slam.robot_pose[2])
        dy = forward_velocity * dt * np.sin(self.slam.robot_pose[2])
        dtheta = angular_velocity * dt
        
        self.slam.update_pose(dx, dy, dtheta)
    
    def obstacle_avoidance(self):
        """
        Simple obstacle avoidance behavior
        
        Returns:
            str: Action to take ('forward', 'turn_left', 'turn_right', 'stop')
        """
        has_obstacle, min_distance = self.lidar.is_obstacle_ahead(
            threshold=self.obstacle_threshold,
            angle_range=(330, 30)
        )
        
        if has_obstacle:
            # Check left and right
            left_distances = self.lidar.get_front_obstacles(angle_range=(30, 90))
            right_distances = self.lidar.get_front_obstacles(angle_range=(270, 330))
            
            left_min = min(left_distances) if left_distances else float('inf')
            right_min = min(right_distances) if right_distances else float('inf')
            
            # Turn towards the side with more space
            if left_min > right_min:
                return 'turn_left'
            else:
                return 'turn_right'
        
        return 'forward'
    
    def execute_action(self, action):
        """Execute motor action"""
        if action == 'forward':
            self.motors.move_forward(self.forward_speed)
            self.current_speed = self.forward_speed
            self.current_angular_velocity = 0.0
        elif action == 'turn_left':
            self.motors.turn_left(self.turn_speed)
            self.current_speed = 0.0
            self.current_angular_velocity = self.turn_speed
        elif action == 'turn_right':
            self.motors.turn_right(self.turn_speed)
            self.current_speed = 0.0
            self.current_angular_velocity = -self.turn_speed
        elif action == 'stop':
            self.motors.stop_all()
            self.current_speed = 0.0
            self.current_angular_velocity = 0.0
    
    def visualization_thread(self):
        """Thread for updating map visualization"""
        if not self.show_map:
            return
        
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        
        while self.running:
            try:
                self.ax.clear()
                self.slam.visualize(ax=self.ax, show_path=True)
                plt.pause(0.1)
            except Exception as e:
                print(f"Visualization error: {e}")
                break
    
    def run(self):
        """Main autonomous loop"""
        self.running = True
        
        # Start visualization thread
        if self.show_map:
            self.viz_thread = Thread(target=self.visualization_thread, daemon=True)
            self.viz_thread.start()
        
        print("\n=== Starting Autonomous SLAM Navigation ===")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.running:
                current_time = time.time()
                dt = current_time - self.last_update_time
                self.last_update_time = current_time
                
                # Update odometry
                self.estimate_odometry(dt)
                
                # Update SLAM map periodically
                if current_time - self.last_scan_time >= self.scan_update_rate:
                    scan = self.lidar.get_latest_scan()
                    if scan is not None:
                        self.slam.update_map(scan)
                        self.last_scan_time = current_time
                
                # Obstacle avoidance
                action = self.obstacle_avoidance()
                self.execute_action(action)
                
                # Print status
                scan = self.lidar.get_latest_scan()
                if scan:
                    has_obs, min_dist = self.lidar.is_obstacle_ahead(
                        threshold=self.obstacle_threshold
                    )
                    print(f"Action: {action:12s} | "
                          f"Front dist: {min_dist:6.0f} mm | "
                          f"Scan points: {scan['count']:4d} | "
                          f"Map updates: {len(self.slam.robot_path)}")
                
                time.sleep(0.05)  # Main loop rate
                
        except KeyboardInterrupt:
            print("\nStopping robot...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        self.running = False
        
        if self.motors:
            self.motors.stop_all()
            self.motors.close()
        
        if self.lidar:
            self.lidar.stop()
        
        if self.slam:
            # Save map
            self.slam.save_map('slam_map.npy')
            print("Map saved to slam_map.npy")
        
        print("Cleanup complete")


def main():
    """Main entry point"""
    # Configuration
    config = {
        'lidar_port': '/dev/ttyUSB0',
        'lidar_pwm': 500,
        'left_pins': (17, 22, 25),
        'right_pins': (23, 24, 5),
        'map_size': 800,
        'map_resolution': 0.05,  # 5cm per pixel
        'obstacle_threshold': 400,  # mm
        'forward_speed': 0.5,
        'turn_speed': 0.5,
        'show_map': True
    }
    
    # Create robot
    robot = AutonomousSlamRobot(config)
    
    # Signal handler
    def signal_handler(sig, frame):
        robot.running = False
        print("\nReceived interrupt signal")
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Initialize and run
        robot.initialize()
        robot.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.cleanup()


if __name__ == '__main__':
    main()

