# RT1-Assignment_2 Amirmohammad Saberi - S5840276
## ROS Node for Goal Management and Data Publishing - Node A

This repository contains a Python script that functions as a ROS (Robot Operating System) node. It's specifically designed for robotics applications requiring goal management and position and velocity data handling.

## How the Script Works

- **Initialization**: The script sets up a ROS publisher to broadcast position and velocity information using a custom message type (`msga`).
- **Action Client**: An action client is initialized to send and manage goals (`PlanningAction`).
- **Odometry Subscription**: The node subscribes to the `/odom` topic to receive odometry data.
- **User Command Processing**: It processes user inputs, allowing for setting or cancelling of goals.
- **Continuous Operation**: The node operates in a loop, continuously handling user commands and updating the status of goals.
- **Goal Management**: Key functionalities include setting new goals based on user inputs, cancelling ongoing goals, and logging the status of current goals, all within the ROS ecosystem.

# ROS Node for Goal Setting and Position Publishing

This Python script is a ROS node designed for setting goals, handling user commands, and publishing position and velocity data. It's part of the `assignment_2_2023` package and utilizes custom message types.

## Script Overview

The script includes functions for initializing a ROS publisher and action client, subscribing to odometry data, processing user commands, setting and cancelling goals, logging goal status, and publishing position and velocity data.

Here's the detailed code with comments explaining each part:

```python
#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import actionlib
import assignment_2_2023.msg
from assignment_2_2023.msg import msga
from assignment_2_2023.msg import PlanningAction, PlanningGoal

def initialize_publisher():
    return rospy.Publisher("/pos_vel", msga, queue_size=1)

def initialize_action_client():
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()
    return client

def subscribe_to_odometry(pub):
    rospy.Subscriber("/odom", Odometry, publish_position_velocity, callback_args=pub)

def process_user_command(client):
    command = input("Press 'y' to set a new goal or 'c' to cancel the current goal: ")
    if command == 'y':
        return set_new_goal(client)
    elif command == 'c':
        return cancel_current_goal(client)
    else:
        rospy.logwarn("Invalid command. Please enter 'y' or 'c'.")
        return None

def set_new_goal(client):
    try:
        input_x = float(input("Enter the x-coordinate for the new goal: "))
        input_y = float(input("Enter the y-coordinate for the new goal: "))
    except ValueError:
        rospy.logwarn("Please enter a valid number.")
        return None

    goal = PlanningGoal()
    goal.target_pose.pose.position.x = input_x
    goal.target_pose.pose.position.y = input_y
    client.send_goal(goal)
    return goal

def cancel_current_goal(client):
    client.cancel_goal()
    rospy.loginfo("Current goal has been cancelled")
    return None

def log_goal_status(goal):
    if goal:
        rospy.loginfo("Current goal: target_x = %f, target_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

def publish_position_velocity(msg, pub):
    position_and_velocity_data = msga()
    position_and_velocity_data.positionx = msg.pose.pose.position.x
    position_and_velocity_data.positiony = msg.pose.pose.position.y
    position_and_velocity_data.velocityx = msg.twist.twist.linear.x
    position_and_velocity_data.velocityz = msg.twist.twist.angular.z
    pub.publish(position_and_velocity_data)

def main():
    rospy.init_node('set_target_client')
    pub = initialize_publisher()
    client = initialize_action_client()
    subscribe_to_odometry(pub)

    while not rospy.is_shutdown():
        goal = process_user_command(client)
        log_goal_status(goal)

if __name__ == '__main__':
    main()
```



