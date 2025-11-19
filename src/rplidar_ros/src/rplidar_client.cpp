/*
 * Custom RPLIDAR ROS Client Node
 * Neovio Autonomous Car Project
 * 
 * Simple test client to subscribe to laser scan data
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    if (!scan) {
        ROS_WARN("Received null scan message");
        return;
    }
    
    int count = scan->ranges.size();
    ROS_INFO("Received laser scan from %s [%d points]", 
             scan->header.frame_id.c_str(), count);
    ROS_INFO("Angle range: %.2f to %.2f degrees", 
             RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    
    // Print first few points
    int max_output = std::min(10, count);
    for (int i = 0; i < max_output; i++) {
        float angle = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        float range = scan->ranges[i];
        ROS_INFO("  [%.2f deg] Range: %.3f m", angle, range);
    }
    
    if (count > max_output) {
        ROS_INFO("  ... (%d more points)", count - max_output);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>(
        "/scan", 1000, scanCallback);
    
    ROS_INFO("RPLIDAR client node started. Listening to /scan topic...");
    
    ros::spin();
    
    return 0;
}

