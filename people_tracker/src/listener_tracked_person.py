#!/usr/bin/env python3

import rospy
import numpy
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseWithCovariance, Pose
from std_msgs.msg import Float32



def newMessageReceived(TrackedPersons):
    
    global poseStamped
    poseStamped = PoseStamped()
    rate = rospy.Rate(1)
    
    for TrackedPerson in TrackedPersons.tracks:

        poseStamped.header.frame_id = "map"
        poseStamped.pose.position = TrackedPerson.pose.pose.position
        poseStamped.pose.orientation = TrackedPerson.pose.pose.orientation
    
    pub_1.publish(poseStamped)    
    rate.sleep()
    
def poseCallback(PoseWithCovarianceStamped):
    
    currPose = Pose()
    
    currPose.position = PoseWithCovarianceStamped.pose.pose.position
    currPose.orientation = PoseWithCovarianceStamped.pose.pose.orientation
    
    diffX = currPose.position.x - poseStamped.pose.position.x
    diffY = currPose.position.y - poseStamped.pose.position.y
    
    dist_to_person = numpy.hypot(diffX, diffY)
    
    pub_2.publish(dist_to_person)
    
    
# Initialize node
rospy.init_node("tracked_persons_to_move_base_goal")


# Create publisher and subscriber
inputTopic_1 = rospy.resolve_name("/spencer/perception/tracked_persons")
inputTopic_2 = rospy.resolve_name("/amcl_pose")
outputTopic_1 = rospy.resolve_name("/move_base_simple/goal")
outputTopic_2 = rospy.resolve_name("/dist_to_person")
sub_1 = rospy.Subscriber(inputTopic_1, TrackedPersons,
                       newMessageReceived, queue_size=20)
sub_2 = rospy.Subscriber(inputTopic_2, PoseWithCovarianceStamped,
                       poseCallback, queue_size=20)
pub_1 = rospy.Publisher(outputTopic_1, PoseStamped, queue_size=20)
pub_2 = rospy.Publisher(outputTopic_2, Float32, queue_size=5)

rospy.spin()
