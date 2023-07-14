import rospy
import roslaunch

if __name__ == '__main__':
    # Initialize the rospy node
    rospy.init_node('roslaunch_example', anonymous=True)

    # Specify the launch file and package
    package = "wheelchair_simulations"
    launch_file = "wheelchair_monte_carlo.launch"

    # Create a roslaunch parent node
    roslaunch_parent = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"), [])

    # Load the launch file
    roslaunch_parent.load([roslaunch.rlutil.resolve_launch_arguments([package, launch_file])])

    # Start the launch file
    roslaunch_parent.start()

    # Wait for the launch process to complete
    rospy.spin()
