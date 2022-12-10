#!/usr/bin/env python3

import rospy
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseStamped, Pose


def newMessageReceived(TrackedPersons):
    poseStamped = PoseStamped()

    for TrackedPerson in TrackedPersons.tracks:

        poseStamped.header = TrackedPersons.header
        poseStamped.pose.position = TrackedPerson.pose.pose.position
        poseStamped.pose.orientation = TrackedPerson.pose.pose.orientation

    pub.publish(poseStamped)
    
    
# Initialize node
rospy.init_node("tracked_persons_to_move_base_goal")


# Create publisher and subscriber
inputTopic = rospy.resolve_name("/spencer/perception/tracked_persons")
outputTopic = rospy.resolve_name("/move_base_simple/goal")
sub = rospy.Subscriber(inputTopic, TrackedPersons,
                       newMessageReceived, queue_size=20)
pub = rospy.Publisher(outputTopic, PoseStamped, queue_size=20)

rospy.spin()

# rate = rospy.Rate(1)

# while not rospy.is_shutdown():
#     pub = rospy.Publisher(outputTopic, PoseStamped, queue_size=20)
#     rate.sleep()
#     rospy.spin()
