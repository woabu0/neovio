#!/usr/bin/env python3

import speech_recognition as sr
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
from geometry_msgs.msg import Twist
import time

class VoiceCommandNode:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Predefined locations (x, y, orientation_w)
        self.locations = {
            "fridge": (2.0, 0.0, 1.0),
            "kitchen": (1.0, 1.0, 1.0),
            "living room": (-1.0, 0.0, 1.0),
            # Add more locations as needed
        }

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Publisher for manual control
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)

        print("Voice command node ready. Say commands like 'go to fridge' or 'move forward'")

    def process_command(self, command):
        command = command.lower()
        print(f"Processing command: {command}")

        # Navigation commands
        if "go to" in command or "go near" in command or "take me to" in command:
            location = None
            for key in self.locations:
                if key in command:
                    location = key
                    break
            if location:
                self.navigate_to_location(location)
            else:
                print(f"Location not recognized. Known locations: {list(self.locations.keys())}")

        # Manual movement commands
        elif "forward" in command:
            twist = Twist()
            twist.linear.x = 0.3
            self.cmd_pub.publish(twist)
            time.sleep(1)  # Move for 1 second
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)

        elif "backward" in command or "back" in command:
            twist = Twist()
            twist.linear.x = -0.3
            self.cmd_pub.publish(twist)
            time.sleep(1)
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)

        elif "turn left" in command:
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_pub.publish(twist)
            time.sleep(1.5)
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

        elif "turn right" in command:
            twist = Twist()
            twist.angular.z = -0.5
            self.cmd_pub.publish(twist)
            time.sleep(1.5)
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

        elif "stop" in command:
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.client.cancel_all_goals()

        # Add more commands as needed
        else:
            print("Command not recognized. Try: 'go to fridge', 'move forward', 'stop', etc.")

    def navigate_to_location(self, location):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.locations[location][0]
        goal.target_pose.pose.position.y = self.locations[location][1]
        goal.target_pose.pose.orientation.w = self.locations[location][2]

        print(f"Navigating to {location}")
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(120.0))  # 2 minute timeout

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            print(f"Successfully reached {location}")
        else:
            print(f"Failed to reach {location}")

if __name__ == '__main__':
    rospy.init_node('voice_command_node')
    node = VoiceCommandNode()

    while not rospy.is_shutdown():
        try:
            with node.microphone as source:
                print("Listening...")
                audio = node.recognizer.listen(source, timeout=5)

            command = node.recognizer.recognize_google(audio)
            node.process_command(command)

        except sr.WaitTimeoutError:
            pass  # No command, continue listening
        except sr.UnknownValueError:
            print("Could not understand audio")
        except sr.RequestError as e:
            print(f"Could not request results; {e}")
        except KeyboardInterrupt:
            break

    print("Voice command node shutting down")
