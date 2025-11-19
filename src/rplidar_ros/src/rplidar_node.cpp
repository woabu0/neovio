/*
 * Custom RPLIDAR ROS Node
 * Neovio Autonomous Car Project
 * 
 * This is a custom implementation for interfacing with RPLIDAR A1 sensor
 * using the SLAMTEC SDK for hardware communication.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

// Include SLAMTEC SDK headers
// Note: The SDK should be available in the system or included as a subdirectory
#ifdef USE_SLAMTEC_SDK
#include "sl_lidar.h"
using namespace sl;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

// Global driver instance
#ifdef USE_SLAMTEC_SDK
ILidarDriver* lidar_driver = nullptr;
IChannel* comm_channel = nullptr;
#endif

/**
 * Publish laser scan data to ROS topic
 */
void publishLaserScan(ros::Publisher* publisher,
                      const std::vector<float>& ranges,
                      const std::vector<float>& intensities,
                      float angle_min, float angle_max,
                      float scan_time, const std::string& frame_id) {
    sensor_msgs::LaserScan scan_msg;
    
    scan_msg.header.stamp = ros::Time::now();
    scan_msg.header.frame_id = frame_id;
    
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
    scan_msg.angle_increment = (angle_max - angle_min) / (ranges.size() - 1);
    scan_msg.time_increment = scan_time / ranges.size();
    scan_msg.scan_time = scan_time;
    scan_msg.range_min = 0.15;  // 15cm minimum range for RPLIDAR A1
    scan_msg.range_max = 6.0;   // 6m maximum range for RPLIDAR A1
    
    scan_msg.ranges = ranges;
    scan_msg.intensities = intensities;
    
    publisher->publish(scan_msg);
}

/**
 * Service callback to stop the motor
 */
bool stopMotorService(std_srvs::Empty::Request& req,
                      std_srvs::Empty::Response& res) {
#ifdef USE_SLAMTEC_SDK
    if (!lidar_driver) {
        ROS_ERROR("Lidar driver not initialized");
        return false;
    }
    
    sl_result result = lidar_driver->setMotorSpeed(0);
    if (SL_IS_FAIL(result)) {
        ROS_ERROR("Failed to stop motor");
        return false;
    }
    ROS_INFO("Motor stopped");
    return true;
#else
    ROS_WARN("SDK not available - motor control not implemented");
    return false;
#endif
}

/**
 * Service callback to start the motor
 */
bool startMotorService(std_srvs::Empty::Request& req,
                       std_srvs::Empty::Response& res) {
#ifdef USE_SLAMTEC_SDK
    if (!lidar_driver) {
        ROS_ERROR("Lidar driver not initialized");
        return false;
    }
    
    if (!lidar_driver->isConnected()) {
        ROS_ERROR("Lidar not connected");
        return false;
    }
    
    sl_result result = lidar_driver->setMotorSpeed();
    if (SL_IS_FAIL(result)) {
        ROS_ERROR("Failed to start motor");
        return false;
    }
    
    result = lidar_driver->startScan(0, 1);
    if (SL_IS_FAIL(result)) {
        ROS_ERROR("Failed to start scan");
        return false;
    }
    
    ROS_INFO("Motor started and scan initiated");
    return true;
#else
    ROS_WARN("SDK not available - motor control not implemented");
    return false;
#endif
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rplidar_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // Get parameters
    std::string serial_port;
    int serial_baudrate = 115200;
    std::string frame_id = "laser";
    bool inverted = false;
    bool angle_compensate = true;
    
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);
    nh_private.param<std::string>("frame_id", frame_id, "laser");
    nh_private.param<bool>("inverted", inverted, false);
    nh_private.param<bool>("angle_compensate", angle_compensate, true);
    
    ROS_INFO("RPLIDAR Node starting...");
    ROS_INFO("Serial port: %s, Baudrate: %d", serial_port.c_str(), serial_baudrate);
    ROS_INFO("Frame ID: %s", frame_id.c_str());
    
    // Create publisher
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    
    // Create services
    ros::ServiceServer stop_motor_srv = nh.advertiseService("stop_motor", stopMotorService);
    ros::ServiceServer start_motor_srv = nh.advertiseService("start_motor", startMotorService);
    
#ifdef USE_SLAMTEC_SDK
    // Initialize SLAMTEC SDK
    auto driver_result = createLidarDriver();
    if (!driver_result) {
        ROS_ERROR("Failed to create lidar driver");
        return -1;
    }
    lidar_driver = *driver_result;
    
    // Create serial channel
    auto channel_result = createSerialPortChannel(serial_port, serial_baudrate);
    if (!channel_result) {
        ROS_ERROR("Failed to create serial port channel");
        delete lidar_driver;
        return -1;
    }
    comm_channel = *channel_result;
    
    // Connect to lidar
    if (SL_IS_FAIL(lidar_driver->connect(comm_channel))) {
        ROS_ERROR("Failed to connect to RPLIDAR at %s", serial_port.c_str());
        delete comm_channel;
        delete lidar_driver;
        return -1;
    }
    
    // Get device info
    sl_lidar_response_device_info_t dev_info;
    if (SL_IS_FAIL(lidar_driver->getDeviceInfo(dev_info))) {
        ROS_ERROR("Failed to get device info");
        lidar_driver->disconnect();
        delete comm_channel;
        delete lidar_driver;
        return -1;
    }
    
    ROS_INFO("RPLIDAR connected successfully");
    
    // Check health
    sl_lidar_response_device_health_t health_info;
    if (SL_IS_OK(lidar_driver->getHealth(health_info))) {
        if (health_info.status == SL_LIDAR_STATUS_ERROR) {
            ROS_ERROR("RPLIDAR health check failed - device error");
            lidar_driver->disconnect();
            delete comm_channel;
            delete lidar_driver;
            return -1;
        }
    }
    
    // Start motor (for A series)
    lidar_driver->setMotorSpeed(600);
    
    // Start scan
    LidarScanMode scan_mode;
    sl_result result = lidar_driver->startScan(false, true, 0, &scan_mode);
    if (SL_IS_FAIL(result)) {
        ROS_ERROR("Failed to start scan");
        lidar_driver->setMotorSpeed(0);
        lidar_driver->disconnect();
        delete comm_channel;
        delete lidar_driver;
        return -1;
    }
    
    ROS_INFO("Scan started successfully");
    
    // Main loop
    ros::Rate rate(10);  // 10 Hz
    while (ros::ok()) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t node_count = sizeof(nodes) / sizeof(nodes[0]);
        
        ros::Time scan_start = ros::Time::now();
        result = lidar_driver->grabScanDataHq(nodes, node_count);
        
        if (SL_IS_OK(result)) {
            result = lidar_driver->ascendScanData(nodes, node_count);
            
            if (SL_IS_OK(result)) {
                std::vector<float> ranges;
                std::vector<float> intensities;
                
                float angle_min = std::numeric_limits<float>::max();
                float angle_max = std::numeric_limits<float>::lowest();
                
                for (size_t i = 0; i < node_count; i++) {
                    if (nodes[i].dist_mm_q2 != 0) {
                        float angle = nodes[i].angle_z_q14 * 90.0f / 16384.0f;
                        float distance = nodes[i].dist_mm_q2 / 4.0f / 1000.0f;  // Convert to meters
                        float intensity = nodes[i].quality >> 2;
                        
                        angle_min = std::min(angle_min, angle);
                        angle_max = std::max(angle_max, angle);
                        
                        ranges.push_back(distance);
                        intensities.push_back(intensity);
                    }
                }
                
                if (!ranges.empty()) {
                    float scan_time = (ros::Time::now() - scan_start).toSec();
                    float angle_min_rad = DEG2RAD(angle_min);
                    float angle_max_rad = DEG2RAD(angle_max);
                    
                    publishLaserScan(&scan_pub, ranges, intensities,
                                   angle_min_rad, angle_max_rad,
                                   scan_time, frame_id);
                }
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    // Cleanup
    if (lidar_driver) {
        lidar_driver->setMotorSpeed(0);
        lidar_driver->stop();
        lidar_driver->disconnect();
        delete lidar_driver;
    }
    if (comm_channel) {
        delete comm_channel;
    }
#else
    ROS_WARN("SLAMTEC SDK not available. This node requires the SDK to function.");
    ROS_WARN("Please install the SLAMTEC RPLIDAR SDK or define USE_SLAMTEC_SDK.");
    
    // Keep node alive for service calls
    ros::spin();
#endif
    
    return 0;
}

