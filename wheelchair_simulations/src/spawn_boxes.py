#!/usr/bin/env python3

import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_model(model_name, sdf_path, x_range, y_range, z_range, size_range, is_cylinder):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        with open(sdf_path, 'r') as f:
            sdf = f.read()

        if is_cylinder:
            # Generate random radius and length
            radius = random.uniform(0.3, 0.7)
            # length = 5

            # Replace the radius and length values in the SDF string
            sdf = sdf.replace('<radius>0.5</radius>', f'<radius>{radius}</radius>')
        else:
            # Generate random size
            size_x = random.uniform(size_range[0], size_range[1])
            if (size_x > 1.15):
                size_y = random.uniform(0.5,0.9)
            elif (size_x < 0.8):
                size_y = random.uniform(1.3, 1.8)
            else:
                size_y = random.uniform(0.9, 1.3)
            # size_y = random.uniform(size_range[0], size_range[1])
            size_z = 2.5

            # Replace the size values in the SDF string
            sdf = sdf.replace('<size>1 1 2.5</size>', f'<size>{size_x} {size_y} {size_z}</size>')
        
        # Generate random position
        pose = Pose()
        pose.position.y = random.uniform(y_range[0], y_range[1])

        # eski kare şeklindeki world için kullanılan kısım
        # if pose.position.y > 18 or pose.position.y < 4:
        #     pose.position.x = random.uniform(-4, -18)
        # else:
        #     pose.position.x = random.uniform(x_range[0], x_range[1])
        pose.position.x = random.uniform(x_range[0], x_range[1])
        
        # Adjust z coordinate based on size
        if is_cylinder:
            pose.position.z = 1.25
        else:
            pose.position.z = 1.25

        resp = spawn_sdf(model_name, sdf, '', pose, 'world')
        if resp.success:
            rospy.loginfo("Spawned model: %s" % model_name)
        else:
            rospy.logwarn("Failed to spawn model: %s" % model_name)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % str(e))



if __name__ == '__main__':
    rospy.init_node('spawn_models')

    # Set the path to your SDF files
    box_sdf_path = '/home/otonom/fgm_ws/src/wheelchair_simulations/src/my_box.sdf'
    cylinder_sdf_path = '/home/otonom/fgm_ws/src/wheelchair_simulations/src/my_cylinder.sdf' 

    # Set the number of models to spawn
    num_cylinders = 3
    num_boxes = 3

    # Set the range of random positions for the models
    x_range = [2, 20.5]
    y_range = [-3.5, 3.5]
    # x_range = [-26, 4] #eski kare world için kullanılan
    # y_range = [-3, 25]
    z_range = [1.25, 1.25]

    # Set the range of random sizes for the models
    size_range = [0.5, 2.1]

    for i in range(num_cylinders):
        model_name = 'cylinder{}'.format(i+1)
        spawn_model(model_name, cylinder_sdf_path, x_range, y_range, z_range, size_range, True)

    for i in range(num_boxes):
        model_name = 'box{}'.format(i+1)
        spawn_model(model_name, box_sdf_path, x_range, y_range, z_range, size_range, False)
