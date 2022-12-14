#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Timm Linder, Social Robotics Lab, University of Freiburg
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
Converts a leg_tracker/PersonArray into spencer_tracking_msgs/DetectedPersons which can be processed using a tracker, or visualized using
spencer_tracking_rviz_plugin. This conversion is lossless except for the co-occurrence matrix and the object identifier string.
"""
import rospy, re
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson
from leg_tracker.msg import PersonArray, Person


def newMessageReceived(PersonArray):
    detectedPersons = DetectedPersons()

    if PersonArray.people:
        detectedPersons.header = PersonArray.header
    else:
        detectedPersons.header = PersonArray.header
        detectedPersons.header.frame_id = "odom"  # hack since frame_id is not known

    global detectionId, detectionIdIncrement
    for Person in PersonArray.people:
        # We assume that all detections have the same frame ID
        assert(detectedPersons.header.frame_id == PersonArray.header.frame_id)

        # Construct DetectedPerson
        detectedPerson = DetectedPerson()
        detectedPerson.modality = str(Person.id)
        detectedPerson.confidence = 0.8
        detectedPerson.pose.pose.position = Person.pose.position

        # Covariance
        # for x in range(0, 3):
            # for y in range(0, 3):
                # detectedPerson.pose.covariance[y*6 + x] = 0.1 * x

        # for i in range(3, 6):
            # detectedPerson.pose.covariance[i*6 + i] = 0.1
            
        # detectedPerson.pose.covariance = [0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1]
        detectedPerson.pose.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999999999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999999999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999999999.0]

       # Detection ID
        if useObjectId:
            match = re.search("[0-9]+", str(Person.id))
            if match:
                detectedPerson.detection_id = int(match.group(0))
        else:
            detectedPerson.detection_id = detectionId
            detectionId += detectionIdIncrement

        detectedPersons.detections.append(detectedPerson)

    pub.publish(detectedPersons)


# Initialize node
rospy.init_node("person_array_to_detected_persons")

# Configurable parameters
detectionId = rospy.get_param("~detection_id_offset", 0)
detectionIdIncrement = rospy.get_param("~detection_id_increment", 1)
covScale = rospy.get_param("~cov_scale", 1.0)
useObjectId = rospy.get_param("~use_object_id", True)


# Create publisher and subscriber
inputTopic = rospy.resolve_name("/spencer/perception_internal/people_detection/laser_front/people_tracked")
outputTopic = rospy.resolve_name("/spencer/perception/detected_persons")
sub = rospy.Subscriber(inputTopic, PersonArray, newMessageReceived, queue_size=5)
pub = rospy.Publisher(outputTopic, DetectedPersons, queue_size=5)

rospy.loginfo("Re-publishing people_msgs/PositionMeasurementArray from %s as spencer_tracking_msgs/DetectedPersons at %s" % (inputTopic, outputTopic) )
rospy.spin()
