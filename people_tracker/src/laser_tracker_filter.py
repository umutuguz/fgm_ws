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
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laserCallback)
        self.theta_subscriber = rospy.Subscriber('/track_theta', Float64, self.thetaCallback)
        self.dist_subscriber = rospy.Subscriber('/dist_to_goal', Float64, self.distCallback)
        self.tracking_status_subscriber = rospy.Subscriber('/tracking_status', Bool, self.trackingCallback)
        self.laserScan = LaserScan()
        for i in range(0, 344):
            self.laserScan.ranges.append(1.0)  
        self.trackTheta = Float64()
        self.trackTheta.data = 90
        self.dist_to_goal = Float64()
        self.tracking_status = Bool()
        self.track_index = 172
        self.filter_indexs = 0
        self.index_pos = 0
        self.index_neg = 0
        self.rate_1 = rospy.Rate(1)
        self.rate_10 = rospy.Rate(10)
    
    def publish_laser(self):
        self.laser_publisher.publish(self.laserScan)
        
    def thetaCallback(self, msg):
        
        self.trackTheta = msg
        
    def laserCallback(self, msg):
        
        self.laserScan = msg
        self.laserScan.ranges = list(msg.ranges)
        
    def distCallback(self, msg):
        
        self.dist_to_goal = msg
    
    def trackingCallback(self, msg):
        
        self.tracking_status = msg
        
    def compute_range_index(self):

        # rospy.loginfo("Track theta is: %f" %self.trackTheta.data)
        # rospy.loginfo("Dist is: %f" %(-self.dist_to_goal.data-0.4))
        self.track_index = 343 - round((self.trackTheta.data - 8.5) * (343.0 / 163.0))
        # rospy.loginfo("Track index is: %d" %self.track_index)

        if self.track_index >= 344:
            self.track_index = 343
        # rospy.loginfo(self.track_index)

    def filter_laser_ranges(self):
        if self.tracking_status.data == True:
            # self.filter_indexs = round((-5.0 * (-self.dist_to_goal.data-0.2)) + 40)
            self.filter_indexs = round((-5.0 * (-self.dist_to_goal.data-0.2)) + 50)
            # rospy.loginfo(self.filter_indexs)
            # rospy.loginfo("Filter indexs is: %f" %self.filter_indexs)
            # rospy.loginfo("Track index is: %f" %self.track_index)
            
            if self.track_index - self.filter_indexs < 0:
                self.track_index = self.filter_indexs
            elif  self.track_index + self.filter_indexs > 343:
                self.track_index = 343 - self.filter_indexs
                
            for counter in range(1, (self.filter_indexs + 1)):
                
                self.index_pos = int(self.track_index + counter)   
                self.index_neg = int(self.track_index - counter)
                # rospy.loginfo("index_pos index is: %d" %self.index_pos)
                # rospy.loginfo("index_neg index is: %d" %self.index_neg)
                self.laserScan.ranges[self.track_index] = math.inf
                self.laserScan.ranges[self.index_pos] = math.inf
                self.laserScan.ranges[self.index_neg] = math.inf
        # else:
            # rospy.loginfo("No need to laser filter. There is no person!")
       
        # self.laser_publisher.publish(self.laserScan)
        
        
if __name__ == '__main__':
    try:
        laser_tracker_filter_obj = laser_tracker_filter()
        rospy.loginfo("Filtered Laser data published!")
        while not rospy.is_shutdown():
            laser_tracker_filter_obj.compute_range_index()
            laser_tracker_filter_obj.filter_laser_ranges()
            laser_tracker_filter_obj.publish_laser()
            # laser_tracker_filter_obj.rate_10.sleep()
    except rospy.ROSInterruptException: pass
        
rospy.spin()