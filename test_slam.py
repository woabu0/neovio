#!/usr/bin/env python3
"""
Test SLAM Mapper with Simulated Data
Useful for testing SLAM without hardware
"""

import numpy as np
import matplotlib.pyplot as plt
from slam_mapper import SLAMMapper
import time

def generate_simulated_scan(robot_x, robot_y, robot_theta, obstacles):
    """
    Generate simulated LiDAR scan
    
    Args:
        robot_x, robot_y: Robot position in meters
        robot_theta: Robot orientation in radians
        obstacles: List of (x, y, radius) tuples
    
    Returns:
        dict: Scan data in same format as LidarHandler
    """
    angles = []
    distances = []
    quality = []
    points = []
    
    # Generate 360-degree scan
    for angle_deg in range(0, 360, 2):
        angle_rad = np.deg2rad(angle_deg)
        
        # Ray direction in world frame
        ray_dir_x = np.cos(robot_theta + angle_rad)
        ray_dir_y = np.sin(robot_theta + angle_rad)
        
        # Find closest intersection with obstacles
        min_distance = 4.0  # Max range in meters
        
        for obs_x, obs_y, radius in obstacles:
            # Ray-sphere intersection
            dx = obs_x - robot_x
            dy = obs_y - robot_y
            
            # Project robot position onto ray
            t = dx * ray_dir_x + dy * ray_dir_y
            
            if t > 0:  # Obstacle is in front
                # Distance from ray to obstacle center
                proj_x = robot_x + t * ray_dir_x
                proj_y = robot_y + t * ray_dir_y
                dist_to_center = np.sqrt((proj_x - obs_x)**2 + (proj_y - obs_y)**2)
                
                if dist_to_center < radius:
                    # Hit the obstacle
                    hit_dist = t - np.sqrt(radius**2 - dist_to_center**2)
                    if 0.1 < hit_dist < min_distance:
                        min_distance = hit_dist
        
        # Convert to mm
        distance_mm = min_distance * 1000
        
        angles.append(angle_deg)
        distances.append(distance_mm)
        quality.append(100)
        
        # Convert to cartesian
        x = distance_mm * np.cos(angle_rad) / 1000.0
        y = distance_mm * np.sin(angle_rad) / 1000.0
        points.append((x, y))
    
    return {
        'angles': np.array(angles),
        'distances': np.array(distances),
        'quality': np.array(quality),
        'points': np.array(points),
        'timestamp': time.time(),
        'count': len(angles)
    }

def main():
    """Test SLAM with simulated data"""
    print("Testing SLAM Mapper with Simulated Data")
    print("=" * 50)
    
    # Initialize SLAM
    slam = SLAMMapper(map_size=800, resolution=0.05, max_range=4000)
    
    # Define obstacles (x, y, radius in meters)
    obstacles = [
        (1.0, 1.0, 0.3),
        (-1.0, 1.5, 0.25),
        (0.5, -1.0, 0.2),
        (-1.5, -0.5, 0.3),
    ]
    
    # Simulate robot movement in a square
    path_points = [
        (0.0, 0.0, 0.0),      # Start
        (1.0, 0.0, 0.0),      # Move forward
        (1.0, 0.0, np.pi/2),  # Turn
        (1.0, 1.0, np.pi/2),  # Move forward
        (1.0, 1.0, np.pi),    # Turn
        (0.0, 1.0, np.pi),    # Move forward
        (0.0, 1.0, -np.pi/2), # Turn
        (0.0, 0.0, -np.pi/2), # Move forward
    ]
    
    print("Simulating robot movement...")
    
    for i, (target_x, target_y, target_theta) in enumerate(path_points):
        # Get current pose
        current_x = (slam.robot_pose[0] - slam.map_center[0]) * slam.resolution
        current_y = (slam.robot_pose[1] - slam.map_center[1]) * slam.resolution
        current_theta = slam.robot_pose[2]
        
        # Move towards target
        steps = 10
        for step in range(steps):
            # Interpolate position
            alpha = (step + 1) / steps
            x = current_x * (1 - alpha) + target_x * alpha
            y = current_y * (1 - alpha) + target_y * alpha
            theta = current_theta * (1 - alpha) + target_theta * alpha
            
            # Update pose
            dx = x - current_x
            dy = y - current_y
            dtheta = theta - current_theta
            
            slam.update_pose(dx, dy, dtheta)
            
            # Generate and process scan
            scan = generate_simulated_scan(x, y, theta, obstacles)
            slam.update_map(scan)
            
            current_x, current_y, current_theta = x, y, theta
        
        print(f"  Step {i+1}/{len(path_points)}: Position ({target_x:.2f}, {target_y:.2f})")
    
    print("\nGenerating visualization...")
    
    # Visualize map
    plt.figure(figsize=(12, 12))
    slam.visualize(show_path=True)
    plt.tight_layout()
    plt.savefig('slam_test_map.png', dpi=150, bbox_inches='tight')
    print("Map saved to slam_test_map.png")
    
    # Save map
    slam.save_map('slam_test_map.npy')
    
    print("\nTest complete!")
    print("Check 'slam_test_map.png' for visualization")
    
    plt.show()

if __name__ == '__main__':
    main()

