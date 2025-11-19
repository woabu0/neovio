#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
import tf
import time

try:
    from gpiozero import Motor, PWMOutputDevice
except ImportError:
    rospy.logwarn("gpiozero not available. Running in simulation mode.")
    Motor = None
    PWMOutputDevice = None

class RobotBaseController:
    def __init__(self):
        rospy.init_node('robot_base_controller')
        
        # Get parameters
        self.left_enable_pin = rospy.get_param('~left_enable_pin', 22)
        self.left_forward_pin = rospy.get_param('~left_forward_pin', 17)
        self.left_backward_pin = rospy.get_param('~left_backward_pin', 27)
        
        self.right_enable_pin = rospy.get_param('~right_enable_pin', 25)
        self.right_forward_pin = rospy.get_param('~right_forward_pin', 23)
        self.right_backward_pin = rospy.get_param('~right_backward_pin', 24)
        
        self.wheel_base = rospy.get_param('~wheel_base', 0.26)  # Distance between wheels in meters
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.065)  # Wheel radius in meters
        self.max_speed = rospy.get_param('~max_speed', 0.8)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()
        
        # Initialize motors if gpiozero is available
        self.simulation_mode = Motor is None
        if not self.simulation_mode:
            try:
                self.left_motor = Motor(forward=self.left_forward_pin, 
                                       backward=self.left_backward_pin, 
                                       enable=self.left_enable_pin, 
                                       pwm=True)
                self.right_motor = Motor(forward=self.right_forward_pin, 
                                        backward=self.right_backward_pin, 
                                        enable=self.right_enable_pin, 
                                        pwm=True)
                rospy.loginfo("Motors initialized successfully")
            except Exception as e:
                rospy.logerr(f"Failed to initialize motors: {e}")
                self.simulation_mode = True
        else:
            rospy.logwarn("Running in simulation mode - motors not initialized")
        
        # Publishers and subscribers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.odom_broadcaster = TransformBroadcaster()
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Timer for odometry publishing
        self.odom_timer = rospy.Timer(rospy.Duration(0.02), self.publish_odometry)  # 50Hz
        
        rospy.loginfo("Robot base controller initialized")
    
    def cmd_vel_callback(self, data):
        linear_x = data.linear.x
        angular_z = data.angular.z
        
        # Differential drive calculation
        # v_left = v - (w * wheel_base / 2)
        # v_right = v + (w * wheel_base / 2)
        left_speed = linear_x - (angular_z * self.wheel_base / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_base / 2.0)
        
        # Normalize speeds to [-1, 1]
        max_spd = max(abs(left_speed), abs(right_speed))
        if max_spd > 1.0:
            left_speed /= max_spd
            right_speed /= max_spd
        
        # Scale to max_speed
        left_speed *= self.max_speed
        right_speed *= self.max_speed
        
        # Control motors
        if not self.simulation_mode:
            try:
                if left_speed > 0.01:
                    self.left_motor.forward(min(abs(left_speed), 1.0))
                elif left_speed < -0.01:
                    self.left_motor.backward(min(abs(left_speed), 1.0))
                else:
                    self.left_motor.stop()
                
                if right_speed > 0.01:
                    self.right_motor.forward(min(abs(right_speed), 1.0))
                elif right_speed < -0.01:
                    self.right_motor.backward(min(abs(right_speed), 1.0))
                else:
                    self.right_motor.stop()
            except Exception as e:
                rospy.logerr(f"Error controlling motors: {e}")
        else:
            # In simulation, just log the speeds
            rospy.logdebug(f"Simulation: left={left_speed:.2f}, right={right_speed:.2f}")
        
        # Update odometry (simplified - in real implementation, use encoder feedback)
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt > 0:
            # Simple odometry update based on commanded velocities
            # In a real robot, you would use encoder feedback
            self.x += linear_x * math.cos(self.theta) * dt
            self.y += linear_x * math.sin(self.theta) * dt
            self.theta += angular_z * dt
            
            # Normalize theta to [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.last_time = current_time
    
    def publish_odometry(self, event):
        current_time = rospy.Time.now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        quat = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Set covariance (adjust based on your robot's accuracy)
        odom.pose.covariance[0] = 0.1  # x
        odom.pose.covariance[7] = 0.1  # y
        odom.pose.covariance[35] = 0.2  # yaw
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Broadcast transform
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            quat,
            current_time,
            self.base_frame,
            self.odom_frame
        )
    
    def cleanup(self):
        if not self.simulation_mode:
            try:
                self.left_motor.stop()
                self.right_motor.stop()
                self.left_motor.close()
                self.right_motor.close()
            except:
                pass

if __name__ == '__main__':
    try:
        controller = RobotBaseController()
        rospy.on_shutdown(controller.cleanup)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

