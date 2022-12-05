#!/usr/bin/env python
import rospy
from jsk_recognition_msgs.msg import BoundingBox
import math
from geometry_msgs.msg import Pose, Vector3
import tf
import std_msgs.msg

"""
# BoundingBox represents a oriented bounding box.                                                                                                                         
Header header                                                                                                                                                             
geometry_msgs/Pose pose                                                                                                                                                   
geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)                                                                                                        
# You can use this field to hold value such as likelihood                                                                                                                 
float32 value                                                                                                                                                             
uint32 label
"""

def dummyBoundingBoxPublisher():
    pub = rospy.Publisher('/dummy_bounding_box', BoundingBox, queue_size=1)
    rospy.init_node('dummyBoundingBoxPublisher_node', anonymous=True)
    rate = rospy.Rate(25)
    
    boundingBox_object = BoundingBox()
    i = 0
    pose_object = Pose()
    dimensions_object = Vector3()
    minimum_dimension = 0.2
    boundingBox_object.label = 1234
    
    while not rospy.is_shutdown():
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        h.frame_id = "world"
        
        boundingBox_object.header = h 
        
        
        sinus_value = math.sin(i/10.0)
        boundingBox_object.value = sinus_value
        
        
        # Change Pose to see effects
        pose_object.position.x = 1.0
        pose_object.position.y = 0.0
        pose_object.position.z = sinus_value
        
        
        # ai, aj, ak == roll, pitch, yaw
        quaternion = tf.transformations.quaternion_from_euler(ai=0, aj=0, ak=sinus_value)
        pose_object.orientation.x = quaternion[0]
        pose_object.orientation.y = quaternion[1]
        pose_object.orientation.z = quaternion[2]
        pose_object.orientation.w = quaternion[3]

        dimensions_object.x = sinus_value/10 + minimum_dimension
        dimensions_object.y = minimum_dimension
        dimensions_object.z = minimum_dimension
        
        # Assign pose and dimension objects
        boundingBox_object.pose = pose_object
        boundingBox_object.dimensions = dimensions_object
        pub.publish(boundingBox_object)
        rate.sleep()
        
        i += 1

if __name__ == '__main__':
    try:
        dummyBoundingBoxPublisher()
    except rospy.ROSInterruptException:
        pass