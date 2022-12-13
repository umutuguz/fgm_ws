#!/usr/bin/env python3

import rospy
import math
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseWithCovariance, Pose
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
class people_tracker():
    
    def __init__(self):
        rospy.init_node("people_tracker")
        self.dist_publisher = rospy.Publisher('/dist_to_goal', Float64, queue_size = 10)
        self.ref_publisher = rospy.Publisher('/setpoint', Float64, queue_size = 10)
        self.theta_publisher = rospy.Publisher('/track_theta', Float64, queue_size = 10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.tracker_subscriber = rospy.Subscriber('/spencer/perception/tracked_persons', TrackedPersons, self.tracker_callback)
        self.poseStamped = PoseStamped()
        self.trackedPersons = TrackedPersons()
        self.poseWithCovarienceStamped = PoseWithCovarianceStamped()
        self.tf_listener = tf.TransformListener()
        self.distToGoal = 0.0
        self.diffX = 0.0
        self.diffY = 0.0
        self.set_point = -1.0
        self.theta = 0.0
        self.theta_laser = 0.0
        self.pose_theta = 0.0
        self.quaternion = (0.0, 0.0, 0.0, 0.0)
        self.trans = Pose()
        self.ctrl_c = False
        self.rate_1 = rospy.Rate(1)
        self.rate_10 = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)
        
    def publish_dist(self):
        
            self.dist_publisher.publish(self.distToGoal)
            # rospy.loginfo("Dist to goal published!" )
            # rospy.loginfo(self.distToGoal)
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
        
        # self.diffX = self.poseWithCovarienceStamped.pose.pose.position.x - self.poseStamped.pose.position.x
        self.diffX = self.poseStamped.pose.position.x - self.poseWithCovarienceStamped.pose.pose.position.x 
        # self.diffY = self.poseWithCovarienceStamped.pose.pose.position.y - self.poseStamped.pose.position.y
        self.diffY = self.poseStamped.pose.position.y - self.poseWithCovarienceStamped.pose.pose.position.y 
        
        self.distToGoal = -(math.hypot(self.diffX, self.diffY))
    
    def compute_goal(self):
        a = 0
        # self.publish_goal()
        
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
        
        # self.theta = self.theta + self.pose_theta
        
        
    # def get_map(self):

    #     # Get the current transform between the odom and base frames
    #     tf_ok = 0
    #     while tf_ok == 0 and not rospy.is_shutdown():
    #         try:
    #             self.tf_listener.waitForTransform('/map', '/odom', rospy.Time(), rospy.Duration(1.0))
    #             tf_ok = 1
    #         except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    #             pass

    #     try:
    #         (self.trans, rot)  = self.tf_listener.lookupTransform('map', 'odom', rospy.Time(0))
    #     except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    #         rospy.loginfo("TF Exception")
    #         return
    #     rospy.loginfo(self.trans.pose.x)
    #     return self.trans
        
if __name__ == '__main__':

    people_tracker_obj = people_tracker()
    while not rospy.is_shutdown():
        people_tracker_obj.compute_dist()
        # people_tracker_obj.compute_goal()
        people_tracker_obj.publish_dist()
        # rospy.loginfo("Dist to goal published!" )
        # people_tracker_obj.publish_goal()
        # rospy.loginfo("Goal published!")
        people_tracker_obj.publish_ref()
        # rospy.loginfo("Setpoint published!")
        people_tracker_obj.compute_theta()
        # rospy.loginfo("Computing theta!")
        people_tracker_obj.publish_theta()
        # rospy.loginfo("Theta published!")
        rospy.loginfo("theta is: %f" %people_tracker_obj.theta)
        rospy.loginfo("pose_theta is: %f" %people_tracker_obj.pose_theta)
        rospy.loginfo("theta_laser is: %f" %people_tracker_obj.theta_laser)
        people_tracker_obj.rate_1.sleep()
        
    
rospy.spin()