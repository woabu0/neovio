#!/usr/bin/env python3
"""
SLAM Implementation using Occupancy Grid Mapping
Creates and updates a map as the robot moves
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter
import time


class SLAMMapper:
    """
    Occupancy Grid SLAM implementation
    Uses inverse sensor model for mapping
    """
    
    def __init__(self, map_size=800, resolution=0.05, max_range=4000):
        """
        Initialize SLAM mapper
        
        Args:
            map_size: Size of map in pixels (map_size x map_size)
            resolution: Resolution in meters per pixel
            max_range: Maximum LiDAR range in mm
        """
        self.map_size = map_size
        self.resolution = resolution  # meters per pixel
        self.max_range = max_range / 1000.0  # Convert mm to meters
        
        # Occupancy grid: 0 = unknown, 1 = occupied, -1 = free
        # Using log-odds representation for better numerical stability
        self.log_odds_map = np.zeros((map_size, map_size), dtype=np.float32)
        
        # Robot pose (x, y, theta) in meters and radians
        self.robot_pose = np.array([map_size // 2, map_size // 2, 0.0])
        
        # Map center in world coordinates
        self.map_center = np.array([map_size // 2, map_size // 2])
        
        # Parameters for inverse sensor model
        self.lo_occ = 0.9   # Log-odds for occupied
        self.lo_free = -0.7  # Log-odds for free
        self.lo_max = 5.0    # Maximum log-odds
        self.lo_min = -5.0   # Minimum log-odds
        
        # History for visualization
        self.robot_path = [self.robot_pose.copy()]
        
    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates"""
        map_x = int(x / self.resolution + self.map_center[0])
        map_y = int(y / self.resolution + self.map_center[1])
        return map_x, map_y
    
    def map_to_world(self, map_x, map_y):
        """Convert map coordinates to world coordinates"""
        x = (map_x - self.map_center[0]) * self.resolution
        y = (map_y - self.map_center[1]) * self.resolution
        return x, y
    
    def update_pose(self, dx, dy, dtheta):
        """
        Update robot pose based on odometry
        
        Args:
            dx: Displacement in x (meters) in robot frame
            dy: Displacement in y (meters) in robot frame
            dtheta: Rotation in radians
        """
        # Current pose in world coordinates
        world_x = (self.robot_pose[0] - self.map_center[0]) * self.resolution
        world_y = (self.robot_pose[1] - self.map_center[1]) * self.resolution
        theta = self.robot_pose[2]
        
        # Transform displacement from robot frame to world frame
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        world_dx = dx * cos_theta - dy * sin_theta
        world_dy = dx * sin_theta + dy * cos_theta
        
        # Update world position
        world_x += world_dx
        world_y += world_dy
        
        # Update angle
        theta += dtheta
        theta = np.arctan2(np.sin(theta), np.cos(theta))  # Normalize to [-pi, pi]
        
        # Convert back to map coordinates
        self.robot_pose[0] = world_x / self.resolution + self.map_center[0]
        self.robot_pose[1] = world_y / self.resolution + self.map_center[1]
        self.robot_pose[2] = theta
        
        # Record path
        self.robot_path.append(self.robot_pose.copy())
    
    def update_map(self, scan_data):
        """
        Update occupancy grid with new scan data
        
        Args:
            scan_data: dict with 'angles', 'distances', 'points' from LidarHandler
        """
        if scan_data is None or scan_data['count'] == 0:
            return
        
        robot_x = self.robot_pose[0]
        robot_y = self.robot_pose[1]
        robot_theta = self.robot_pose[2]
        
        # Get scan points in world frame
        angles = np.deg2rad(scan_data['angles'])
        distances = scan_data['distances'] / 1000.0  # Convert mm to meters
        
        # Transform points to map frame
        for angle, distance in zip(angles, distances):
            if distance > self.max_range or distance < 0.1:
                continue
            
            # Point in robot frame
            local_x = distance * np.cos(angle)
            local_y = distance * np.sin(angle)
            
            # Transform to map frame
            cos_theta = np.cos(robot_theta)
            sin_theta = np.sin(robot_theta)
            
            map_x = robot_x + local_x * cos_theta - local_y * sin_theta
            map_y = robot_y + local_x * sin_theta + local_y * cos_theta
            
            # Mark occupied cell
            if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
                self.log_odds_map[int(map_y), int(map_x)] += self.lo_occ
                self.log_odds_map[int(map_y), int(map_x)] = np.clip(
                    self.log_odds_map[int(map_y), int(map_x)],
                    self.lo_min, self.lo_max
                )
            
            # Mark free cells along the ray
            self._mark_free_cells(robot_x, robot_y, map_x, map_y, distance)
    
    def _mark_free_cells(self, x0, y0, x1, y1, max_dist):
        """Mark cells along ray as free using Bresenham's line algorithm"""
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        steps = 0
        max_steps = int(max_dist / self.resolution)
        
        while True:
            if 0 <= x < self.map_size and 0 <= y < self.map_size:
                # Don't mark the endpoint as free (it's occupied)
                if steps < max_steps - 1:
                    self.log_odds_map[y, x] += self.lo_free
                    self.log_odds_map[y, x] = np.clip(
                        self.log_odds_map[y, x],
                        self.lo_min, self.lo_max
                    )
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
            
            steps += 1
            if steps >= max_steps:
                break
    
    def get_map(self):
        """
        Get occupancy grid as probability map
        
        Returns:
            2D numpy array: 0 = free, 1 = occupied, 0.5 = unknown
        """
        # Convert log-odds to probabilities
        prob_map = 1.0 / (1.0 + np.exp(-self.log_odds_map))
        
        # Apply threshold
        occupied = prob_map > 0.6
        free = prob_map < 0.4
        
        result = np.full_like(prob_map, 0.5, dtype=np.float32)  # Unknown
        result[occupied] = 1.0  # Occupied
        result[free] = 0.0      # Free
        
        return result
    
    def get_visualization_map(self):
        """
        Get map for visualization with robot path
        
        Returns:
            2D numpy array ready for matplotlib
        """
        map_img = self.get_map()
        
        # Apply smoothing for better visualization
        map_img = gaussian_filter(map_img, sigma=0.5)
        
        return map_img
    
    def visualize(self, ax=None, show_path=True):
        """
        Visualize the map
        
        Args:
            ax: Matplotlib axis (if None, creates new figure)
            show_path: Whether to show robot path
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 10))
        
        map_img = self.get_visualization_map()
        
        # Display map
        ax.imshow(map_img, cmap='gray_r', origin='lower', vmin=0, vmax=1)
        
        # Draw robot path
        if show_path and len(self.robot_path) > 1:
            path = np.array(self.robot_path)
            ax.plot(path[:, 0], path[:, 1], 'r-', linewidth=2, alpha=0.6, label='Robot Path')
        
        # Draw current robot position
        if len(self.robot_path) > 0:
            robot = self.robot_path[-1]
            ax.plot(robot[0], robot[1], 'ro', markersize=10, label='Robot')
            
            # Draw robot orientation
            arrow_length = 20
            dx = arrow_length * np.cos(robot[2])
            dy = arrow_length * np.sin(robot[2])
            ax.arrow(robot[0], robot[1], dx, dy, head_width=5, head_length=5, 
                    fc='red', ec='red')
        
        ax.set_title('SLAM Occupancy Grid Map')
        ax.set_xlabel('X (pixels)')
        ax.set_ylabel('Y (pixels)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        return ax
    
    def save_map(self, filename):
        """Save map to file"""
        map_img = self.get_map()
        np.save(filename, map_img)
        print(f"Map saved to {filename}")
    
    def load_map(self, filename):
        """Load map from file"""
        map_img = np.load(filename)
        # Convert back to log-odds (approximate)
        self.log_odds_map = np.log(map_img / (1.0 - map_img + 1e-10))
        self.log_odds_map = np.clip(self.log_odds_map, self.lo_min, self.lo_max)
        print(f"Map loaded from {filename}")

