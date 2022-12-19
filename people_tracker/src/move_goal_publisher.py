#!/usr/bin/env python3

import rospy
import math
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseWithCovariance, Pose
from std_msgs.msg import Float64, Bool
from tf.transformations import euler_from_quaternion
class move_goal_publisher():
    
    def __init__(self):
        rospy.init_node("move_goal_publisher")
        self.pose_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
        self.pose_subscriber = rospy.Subscriber('/move_base_simple/goal_pub', PoseStamped, self.pose_callback)
        self.poseStamped = PoseStamped()
        self.rate_1 = rospy.Rate(1)
        self.rate_5 = rospy.Rate(5)
        self.rate_10 = rospy.Rate(10)
        
    def publish_pose(self):
        
        self.pose_publisher.publish(self.poseStamped)
        
    def pose_callback(self,msg):
        
        self.poseStamped = msg
        
if __name__ == '__main__':
    
    try:
        move_goal_publisher_obj = move_goal_publisher()
        while not rospy.is_shutdown():
            move_goal_publisher_obj.publish_pose()
            move_goal_publisher_obj.rate_5.sleep()
    except rospy.ROSInterruptException: pass
            
rospy.spin()