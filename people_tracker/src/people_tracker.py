#!/usr/bin/env python3

import rospy
import math
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseWithCovariance, Pose
from std_msgs.msg import Float64, Bool
from tf.transformations import euler_from_quaternion
class people_tracker():
    
    def __init__(self):
        rospy.init_node("people_tracker")
        self.dist_publisher = rospy.Publisher('/dist_to_goal', Float64, queue_size = 10)
        self.ref_publisher = rospy.Publisher('/setpoint', Float64, queue_size = 10)
        self.theta_publisher = rospy.Publisher('/track_theta', Float64, queue_size = 10)
        self.tracking_publisher = rospy.Publisher('/tracking_status', Bool, queue_size = 10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal_pub', PoseStamped, queue_size = 10)
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.tracker_subscriber = rospy.Subscriber('/tracked_persons', TrackedPersons, self.tracker_callback)
        self.poseStamped = PoseStamped()
        self.trackedPersons = TrackedPersons()
        self.poseWithCovarienceStamped = PoseWithCovarianceStamped()
        self.distToGoal = 0.0
        self.tracking_status = False
        self.diffX = 0.0
        self.diffY = 0.0
        self.set_point = -1.5
        self.theta = 0.0
        self.theta_laser = 0.0
        self.pose_theta = 0.0
        self.quaternion = (0.0, 0.0, 0.0, 0.0)
        self.rate_1 = rospy.Rate(1)
        self.rate_5 = rospy.Rate(5)
        self.rate_10 = rospy.Rate(10)
        
    def publish_dist(self):
        
            self.dist_publisher.publish(self.distToGoal)
            # rospy.loginfo("Dist to goal published!" )
            # self.rate_10.sleep()
            
    def publish_goal(self):
        
            self.goal_publisher.publish(self.poseStamped)
            # rospy.loginfo("Goal published!")
            # self.rate_10.sleep()
    
    def publish_ref(self):
        
            self.ref_publisher.publish(self.set_point)
            # rospy.loginfo("Setpoint published!")
            # self.rate_10.sleep()
    
    def publish_theta(self):
        
            self.theta_publisher.publish(self.theta_laser)
            # rospy.loginfo("Setpoint published!")
            # self.rate_10.sleep()
    
    def publish_tracking_status(self):
        
            self.tracking_publisher.publish(self.tracking_status)
            # rospy.loginfo("Setpoint published!")
            # self.rate_10.sleep()
    
    def shutdownhook(self):
        self.ctrl_c = True
        
    def pose_callback(self, msg):
        self.poseWithCovarienceStamped = msg
    
    def tracker_callback(self,msg):

        self.poseStamped.header = msg.header
        for TrackedPerson in msg.tracks:
            if TrackedPerson.is_matched == True and TrackedPerson.track_id == 0:
                # if TrackedPerson.track_id == 0:
                self.tracking_status = True
                self.poseStamped.pose.position = TrackedPerson.pose.pose.position
                self.poseStamped.pose.orientation = TrackedPerson.pose.pose.orientation
                # rospy.loginfo(TrackedPerson.track_id)
            elif TrackedPerson.is_matched == True and TrackedPerson.track_id != 0:
                self.tracking_status = True
                # self.poseStamped.pose.position = TrackedPerson.pose.pose.position
                # self.poseStamped.pose.orientation = TrackedPerson.pose.pose.orientation
                # rospy.loginfo(TrackedPerson.track_id)
            
            # rospy.loginfo(TrackedPerson.track_id)
    
    def compute_dist(self):
        
        self.diffX = self.poseStamped.pose.position.x - self.poseWithCovarienceStamped.pose.pose.position.x 
        self.diffY = self.poseStamped.pose.position.y - self.poseWithCovarienceStamped.pose.pose.position.y 
        if self.tracking_status == True:
            self.distToGoal = -(math.hypot(self.diffX, self.diffY))
        else:
            self.distToGoal = 0.0
    
    def compute_goal(self):
        a = 0
        
    def compute_theta(self):
        
        self.quaternion = (
            self.poseWithCovarienceStamped.pose.pose.orientation.x,
            self.poseWithCovarienceStamped.pose.pose.orientation.y,
            self.poseWithCovarienceStamped.pose.pose.orientation.z,
            self.poseWithCovarienceStamped.pose.pose.orientation.w)
        self.theta = math.atan2(self.diffY, self.diffX) * (180.0 / math.pi)
        self.pose_theta = tf.transformations.euler_from_quaternion(self.quaternion)
        self.pose_theta = self.pose_theta[2] * (180.0 / math.pi)
        
        if ((90 < self.pose_theta) and (self.pose_theta < 180)):
            self.pose_theta = 450 - self.pose_theta
        else:
            self.pose_theta = 90 - self.pose_theta
            
        if ((self.diffX > 0) and (self.diffY > 0)):
            self.theta = 450 - self.theta
        else:
            self.theta = 90 - self.theta
            
        self.theta_laser = self.theta - self.pose_theta + 90.0
        
        if self.theta_laser < 0:
            self.theta_laser = self.theta_laser + 360
        elif self.theta_laser > 270:
            self.theta_laser = self.theta_laser - 360
        
if __name__ == '__main__':
    
    try:
        people_tracker_obj = people_tracker()
        while not rospy.is_shutdown():
            people_tracker_obj.compute_dist()
            # people_tracker_obj.compute_goal()
            people_tracker_obj.publish_dist()
            if people_tracker_obj.tracking_status == True:
                people_tracker_obj.publish_goal()
            #     # rospy.loginfo("Goal published!")
            # else:
            #     # rospy.loginfo("There is no person to track!")
            people_tracker_obj.publish_ref()
            # rospy.loginfo("Setpoint published!")
            people_tracker_obj.compute_theta()
            # rospy.loginfo("Computing theta!")
            people_tracker_obj.publish_theta()
            # rospy.loginfo("Theta published!")
            people_tracker_obj.publish_tracking_status()
            # people_tracker_obj.rate_10.sleep()
    except rospy.ROSInterruptException: pass
            
rospy.spin()