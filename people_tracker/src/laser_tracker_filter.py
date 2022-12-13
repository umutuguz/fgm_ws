#!/usr/bin/env python3

import rospy
import math
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseWithCovariance, Pose
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import LaserScan

class laser_tracker_filter():
    
    def __init__(self):
        rospy.init_node("laser_tracker_filter")
        self.laser_publisher = rospy.Publisher('/scan_track_filtered', LaserScan, queue_size = 10)
        # self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laserCallback)
        self.laser_subscriber = rospy.Subscriber('/front_rp/rp_scan_filtered_front', LaserScan, self.laserCallback)
        self.theta_subscriber = rospy.Subscriber('/track_theta', Float64, self.thetaCallback)
        self.theta_subscriber = rospy.Subscriber('/dist_to_goal', Float64, self.distCallback)
        self.tracking_status_subscriber = rospy.Subscriber('/tracking_status', Bool, self.trackingCallback)
        self.laserScan = LaserScan()
        self.trackTheta = 0.0
        self.track_index = 0
        self.dist_to_goal = 0.0
        self.filter_indexs = 0
        self.counter = 0
        self.index_pos = 0
        self.index_neg = 0
        self.tracking_status = False
    
    def publish_laser(self):
        
        self.laser_publisher.publish(self.laserScan)
        
    def thetaCallback(self, msg):
        
        self.trackTheta = msg

    def laserCallback(self, msg):
        
        self.laserScan = msg
        
    def distCallback(self, msg):
        
        self.dist_to_goal = msg
    
    def trackingCallback(self, msg):
        
        self.tracking_status = msg
        
    def compute_range_index(self):
        
        self.track_index = round((self.trackTheta - 8.5) * (344.0 / 163.0))

        
        if self.track_index <= 344:
            self.track_index = 343

    def filter_laser_ranges(self):
        
        if self.tracking_status == True:
            
            self.filter_indexs = round((-2.5 * self.dist_to_goal) + 17)
            
            for self.counter in range(1, (self.filter_indexs + 1)):
                self.index_pos = self.track_index + self.counter
                self.index_neg = self.track_index - self.counter
                # self.laserScan.ranges[self.index_pos] = math.inf
                self.laserScan.ranges[self.index_pos] = 99
                # self.laserScan.ranges[self.index_neg] = math.inf
                self.laserScan.ranges[self.index_neg] = 99
                # self.laserScan.ranges[self.track_index] = math.inf
                self.laserScan.ranges[self.track_index] = 99
                rospy.loginfo(self.laserScan.ranges[self.track_index])
            
        

if __name__ == '__main__':
    
    laser_tracker_filter_obj = laser_tracker_filter()
    rospy.loginfo("Filtered Laser data published!")
    while not rospy.is_shutdown():
        laser_tracker_filter_obj.compute_range_index()
        laser_tracker_filter_obj.filter_laser_ranges()
        laser_tracker_filter_obj.publish_laser()
            
        
rospy.spin()