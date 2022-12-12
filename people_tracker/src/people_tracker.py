#!/usr/bin/env python3

import rospy
import numpy
import time
import math
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseWithCovariance, Pose
from std_msgs.msg import Float64

class people_tracker():
    
    def __init__(self):
        rospy.init_node("people_tracker")
        self.dist_publisher = rospy.Publisher('/dist_to_goal', Float64, queue_size = 10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.tracker_subscriber = rospy.Subscriber('/spencer/perception/tracked_persons', TrackedPersons, self.tracker_callback)
        self.poseStamped = PoseStamped()
        self.trackedPersons = TrackedPersons()
        self.poseWithCovarienceStamped = PoseWithCovarianceStamped()
        self.distToGoal = 0.0
        self.diffX = 0.0
        self.diffY = 0.0
        self.trans = Pose()
        self.map_frame = '/map'
        self.ctrl_c = False
        self.rate_1 = rospy.Rate(1)
        self.rate_10 = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)
        
    def publish_dist(self):
        
            self.dist_publisher.publish(self.distToGoal)
            rospy.loginfo("Dist to goal published!" )
            rospy.loginfo(self.distToGoal)
            self.rate_10.sleep()
            
    def publish_goal(self):
        
        # while not self.ctrl_c:
            self.goal_publisher.publish(self.poseStamped)
            rospy.loginfo("Goal published!")
            self.rate_10.sleep()
    
    def shutdownhook(self):
        self.ctrl_c = True
        
    def pose_callback(self, msg):
        self.poseWithCovarienceStamped = msg
    
    def tracker_callback(self,msg):

        for TrackedPerson in msg.tracks:
            self.poseStamped.header = msg.header
            self.poseStamped.pose.position = TrackedPerson.pose.pose.position
            self.poseStamped.pose.orientation = TrackedPerson.pose.pose.orientation
    
    def compute_dist(self):
        
        self.diffX = abs(self.poseStamped.pose.position.x - self.poseWithCovarienceStamped.pose.pose.position.x)
        self.diffY = abs(self.poseStamped.pose.position.y - self.poseWithCovarienceStamped.pose.pose.position.y)
        
        self.distToGoal = math.hypot(self.diffX, self.diffY)
    
        
        
    def compute_goal(self):
        a = 0
        # self.publish_goal()
        
    def get_map(self):

        # Get the current transform between the odom and base frames
        tf_ok = 0
        while tf_ok == 0 and not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform('/map', '/odom', rospy.Time(), rospy.Duration(1.0))
                tf_ok = 1
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                pass

        try:
            (self.trans, rot)  = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return self.trans
        

if __name__ == '__main__':

   people_tracker_obj = people_tracker()
   while not rospy.is_shutdown():
    people_tracker_obj.compute_dist()
    people_tracker_obj.compute_goal()
    people_tracker_obj.publish_dist()
    people_tracker_obj.publish_goal()
    
    # people_tracker_obj.rate_1.sleep()
    
rospy.spin()