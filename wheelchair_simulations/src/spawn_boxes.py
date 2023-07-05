#!/usr/bin/env python3

import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_box(model_name, sdf_path, x_range, y_range, z_range, size_range):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        with open(sdf_path, 'r') as f:
            sdf = f.read()
        
        # Generate random size
        size_x = random.uniform(size_range[0], size_range[1])
        size_y = random.uniform(size_range[0], size_range[1])
        size_z = 2.5

        # Replace the size values in the SDF string
        sdf = sdf.replace('<size>1 1 2.5</size>', f'<size>{size_x} {size_y} {size_z}</size>')

        # Generate random position
        pose = Pose()
        pose.position.x = random.uniform(x_range[0], x_range[1])
        pose.position.y = random.uniform(y_range[0], y_range[1])
        pose.position.z = random.uniform(z_range[0], z_range[1])

        resp = spawn_sdf(model_name, sdf, '', pose, 'world')
        if resp.success:
            rospy.loginfo("Spawned model: %s" % model_name)
        else:
            rospy.logwarn("Failed to spawn model: %s" % model_name)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % str(e))

if __name__ == '__main__':
    rospy.init_node('spawn_boxes')

    # Set the path to your SDF file
    sdf_path = '/home/otonom/fgm_ws/src/wheelchair_simulations/src/my_box.sdf'

    # Set the number of boxes to spawn
    num_boxes = 100

    # Set the range of random positions for the boxes
    x_range = [-50, 50]
    y_range = [-30, 30]
    z_range = [1.25, 1.25]

    # Set the range of random sizes for the boxes
    size_range = [0.5, 2.0]

    for i in range(num_boxes):
        box_name = 'box{}'.format(i+1)
        spawn_box(box_name, sdf_path, x_range, y_range, z_range, size_range)
