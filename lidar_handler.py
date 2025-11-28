#!/usr/bin/env python3
"""
Improved LiDAR Handler with Buffer Management
Fixes max buffer overflow issues by using threading and efficient data processing
"""

import threading
import queue
import time
import numpy as np
from collections import deque
from rplidar import RPLidar


class LidarHandler:
    """Thread-safe LiDAR handler with buffer management"""
    
    def __init__(self, port='/dev/ttyUSB0', pwm_speed=500, max_buffer=3600):
        """
        Initialize LiDAR handler
        
        Args:
            port: Serial port path
            pwm_speed: Motor PWM speed (300-600 recommended)
            max_buffer: Maximum buffer size for measurements
        """
        self.port = port
        self.pwm_speed = pwm_speed
        self.max_buffer = max_buffer
        self.lidar = None
        self.running = False
        self.scan_queue = queue.Queue(maxsize=5)  # Keep only last 5 scans
        self.latest_scan = None
        self.scan_lock = threading.Lock()
        self.thread = None
        
    def _lidar_thread(self):
        """Background thread for reading LiDAR data"""
        try:
            self.lidar = RPLidar(self.port)
            self.lidar._set_pwm(self.pwm_speed)
            time.sleep(1.5)  # Give motor time to spin up
            
            print(f"LiDAR initialized on {self.port}")
            
            # Use iter_scans with proper buffer management
            for scan in self.lidar.iter_scans(max_buf_meas=self.max_buffer):
                if not self.running:
                    break
                
                # Process scan data
                processed_scan = self._process_scan(scan)
                
                # Update latest scan
                with self.scan_lock:
                    self.latest_scan = processed_scan
                
                # Add to queue (drop old scans if queue is full)
                try:
                    self.scan_queue.put_nowait(processed_scan)
                except queue.Full:
                    # Remove oldest and add new
                    try:
                        self.scan_queue.get_nowait()
                        self.scan_queue.put_nowait(processed_scan)
                    except queue.Empty:
                        pass
                        
        except Exception as e:
            print(f"LiDAR thread error: {e}")
        finally:
            self._cleanup()
    
    def _process_scan(self, scan):
        """
        Process raw scan data into structured format
        
        Returns:
            dict with 'angles', 'distances', 'quality', 'points'
        """
        angles = []
        distances = []
        quality = []
        points = []  # (x, y) coordinates
        
        for quality_val, angle, distance in scan:
            if distance > 0:  # Filter invalid readings
                angles.append(angle)
                distances.append(distance)
                quality.append(quality_val)
                
                # Convert to cartesian coordinates
                angle_rad = np.deg2rad(angle)
                x = distance * np.cos(angle_rad)
                y = distance * np.sin(angle_rad)
                points.append((x, y))
        
        return {
            'angles': np.array(angles),
            'distances': np.array(distances),
            'quality': np.array(quality),
            'points': np.array(points),
            'timestamp': time.time(),
            'count': len(angles)
        }
    
    def start(self):
        """Start LiDAR scanning"""
        if self.running:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._lidar_thread, daemon=True)
        self.thread.start()
        
        # Wait for first scan
        timeout = 10
        start_time = time.time()
        while self.latest_scan is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if self.latest_scan is None:
            raise RuntimeError("Failed to get initial LiDAR scan")
        
        print("LiDAR started successfully")
    
    def stop(self):
        """Stop LiDAR scanning"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=5)
        self._cleanup()
    
    def _cleanup(self):
        """Clean up LiDAR resources"""
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass
            self.lidar = None
    
    def get_latest_scan(self):
        """Get the latest scan data (thread-safe)"""
        with self.scan_lock:
            return self.latest_scan
    
    def get_scan_queue(self):
        """Get all scans in queue"""
        scans = []
        while not self.scan_queue.empty():
            try:
                scans.append(self.scan_queue.get_nowait())
            except queue.Empty:
                break
        return scans
    
    def get_front_obstacles(self, angle_range=(330, 30), min_distance=0, max_distance=4000):
        """
        Get obstacles in front of robot
        
        Args:
            angle_range: Tuple of (min_angle, max_angle) in degrees
            min_distance: Minimum distance to consider (mm)
            max_distance: Maximum distance to consider (mm)
        
        Returns:
            List of distances in front sector
        """
        scan = self.get_latest_scan()
        if scan is None:
            return []
        
        min_angle, max_angle = angle_range
        front_distances = []
        
        for angle, distance in zip(scan['angles'], scan['distances']):
            # Handle wrap-around (e.g., 330-360 and 0-30)
            in_range = False
            if min_angle > max_angle:  # Wrap-around case
                in_range = (angle >= min_angle) or (angle <= max_angle)
            else:
                in_range = min_angle <= angle <= max_angle
            
            if in_range and min_distance <= distance <= max_distance:
                front_distances.append(distance)
        
        return front_distances
    
    def is_obstacle_ahead(self, threshold=400, angle_range=(330, 30)):
        """
        Check if obstacle is ahead
        
        Args:
            threshold: Distance threshold in mm
            angle_range: Angle range to check
        
        Returns:
            (bool, float): (has_obstacle, min_distance)
        """
        front_distances = self.get_front_obstacles(angle_range, max_distance=threshold*3)
        
        if not front_distances:
            return False, float('inf')
        
        min_distance = min(front_distances)
        return min_distance < threshold, min_distance

