#!/usr/bin/env python3

import rospy, re
# from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from geometry_msgs.msg import PoseStamped, Pose


def newMessageReceived(TrackedPersons):
    poseStamped = PoseStamped()

    # if TrackedPersons.tracks:
    #     PoseStamped.header = TrackedPersons.header
    # else:
    #     poseStamped.header = TrackedPersons.header
    #     poseStamped.header.frame_id = "odom"  # hack since frame_id is not known

    global detectionId, detectionIdIncrement
    for TrackedPerson in TrackedPersons.tracks:
        # We assume that all detections have the same frame ID
        # assert(poseStamped.header.frame_id == TrackedPersons.header.frame_id)

        # Construct PoseStamped's Pose
        pose = Pose()
        poseStamped.pose.position = TrackedPerson.pose.pose.position
        poseStamped.pose.orientation = TrackedPerson.pose.pose.orientation


        # poseStamped.pose.append(pose)

    pub.publish(poseStamped)


# Initialize node
rospy.init_node("tracked_persons_to_move_base_goal")

# Create publisher and subscriber
# inputTopic = rospy.resolve_name("/spencer/perception_internal/people_detection/laser_front/people_tracked")
inputTopic = rospy.resolve_name("/spencer/perception/tracked_persons")
outputTopic = rospy.resolve_name("/move_base_simple/goal")
sub = rospy.Subscriber(inputTopic, TrackedPersons, newMessageReceived, queue_size=20)
pub = rospy.Publisher(outputTopic, PoseStamped, queue_size=20)

rospy.loginfo("Deneme")
rospy.spin()
