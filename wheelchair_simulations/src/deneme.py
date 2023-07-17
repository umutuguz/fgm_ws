import rospy
import subprocess

if __name__ == '__main__':
    # Initialize the rospy node
    rospy.init_node('roslaunch_example', anonymous=True)

    # Specify the launch file and package
    launch_file = "my_package my_launch_file.launch"

    # Run the roslaunch command
    roslaunch_cmd = ["roslaunch", launch_file]
    subprocess.Popen(roslaunch_cmd)

    # Wait for the launch process to complete
    rospy.spin()
