import rospy
import os
import time
import re
import signal
import subprocess
from move_base_msgs.msg import MoveBaseActionGoal

def get_world_number(launch_file_path):
    # Extract the world number from the launch file
    with open(launch_file_path, 'r') as launch_file:
        launch_content = launch_file.read()

    # Use regular expressions to find the world number
    match = re.search(r"montecarloworld_(\d+)", launch_content)
    if match:
        world_number = match.group(1)
    else:
        world_number = "unknown"

    return world_number

def publish_goal(world_number):
    # Create a publisher to send the navigation goal
    goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    rospy.init_node('goal_publisher', anonymous=True)

    # Create a navigation goal message
    goal_msg = MoveBaseActionGoal()
    goal_msg.goal.target_pose.header.frame_id = "map"
    goal_msg.goal.target_pose.pose.position.x = -23.0
    goal_msg.goal.target_pose.pose.position.y = 23.0
    goal_msg.goal.target_pose.pose.orientation.z = 0.0
    goal_msg.goal.target_pose.pose.orientation.w = 1.0

    rospy.sleep(1)

    # Publish the goal
    goal_pub.publish(goal_msg)

    # Log the successful goal publication
    log_entry = "Goal successfully published at ({}, {}) in world {}".format(
        goal_msg.goal.target_pose.pose.position.x,
        goal_msg.goal.target_pose.pose.position.y,
        world_number
    )
    log_file_path = '/home/otonom/fgm_ws/src/log/mylogs.txt'
    with open(log_file_path, 'a') as log_file:
        log_file.write(log_entry + '\n')

def check_log_file(log_file_path, world_number):
    # Read the log file content
    with open(log_file_path, 'r') as log_file:
        log_content = log_file.read()

    # Find the last occurrence of the world number in the log
    last_world_occurrence = log_content.rfind('world {}'.format(world_number))

    # Check for additional content after the last logged world number
    if last_world_occurrence != -1:
        additional_content = log_content[last_world_occurrence + len('world {}'.format(world_number)):].strip()
        if additional_content:
            # Terminate the remaining terminals by sending SIGINT signals
            script_pid = os.getpid()  # Get the current script's PID
            process_ids = subprocess.check_output(["pgrep", "gnome-terminal"]).splitlines()  # Find process IDs of gnome-terminal
            for pid in process_ids:
                pid = int(pid)
                if pid != script_pid:
                    os.kill(pid, signal.SIGINT)
            time.sleep(10)  # Wait for 10 seconds
            script_path = '/home/otonom/fgm_ws/src/wheelchair_simulations/src/automatesimulations.py'  # Update with the absolute path of your script
            os.system("python3 {}".format(script_path))  # Rerun the script

def launch_file(file_path):
    # Launch the file using roslaunch command in a new terminal
    command = "roslaunch {}".format(file_path)
    os.system("gnome-terminal --command='{}'".format(command))

if __name__ == '__main__':
    # Launch the first launch file
    first_launch_file = '/home/otonom/fgm_ws/src/wheelchair_simulations/launch/wheelchair_monte_carlo.launch'
    launch_file(first_launch_file)
    time.sleep(7)

    # Extract the world number from the first launch file
    world_number = get_world_number(first_launch_file)

    # Launch the second launch file in a new terminal
    second_launch_file = '/home/otonom/fgm_ws/src/wheelchair_simulations/launch/wheelchair_navigation.launch'
    launch_file(second_launch_file)

    time.sleep(5)

    # Publish the navigation goal with the world number
    publish_goal(world_number)

    # Continuously check the log file for additional content until the simulation ends
    log_file_path = '/home/otonom/fgm_ws/src/log/mylogs.txt'
    while not rospy.is_shutdown():
        check_log_file(log_file_path, world_number)
        time.sleep(1)
