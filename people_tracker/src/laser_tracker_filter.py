#!/usr/bin/env python3

import rospy
import math
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseWithCovariance, Pose
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class laser_tracker_filter():
    
    def __init__(self):
        rospy.init_node("laser_tracker_filter")
        