#!/usr/bin/env python
import sys
import rospy
import math
import tf
from geometry_msgs.msg import Vector3

if __name__ == '__main__':
    rospy.init_node('tf_listener_haro_world')

    listener = tf.TransformListener()

    if len(sys.argv) < 3:
        print("usage: tf_world_listener.py parent child")
    else:
        follower_model_name = sys.argv[1]
        model_to_be_followed_name = sys.argv[2]
        
        haro_world_tf_translation = rospy.Publisher('/haro_world_baselink_tf_translation', Vector3 ,queue_size=1)
        
        rate = rospy.Rate(10.0)
        ctrl_c = False
        
        follower_model_frame = "/"+follower_model_name
        model_to_be_followed_frame = "/"+model_to_be_followed_name
        
        def shutdownhook():
            # works better than the rospy.is_shut_down()
            global ctrl_c
            ctrl_c = True

        rospy.on_shutdown(shutdownhook)
        
        while not ctrl_c:
            try:
                (trans,rot) = listener.lookupTransform(follower_model_frame, model_to_be_followed_frame, rospy.Time(0))
                translation_object = Vector3()
                translation_object.x = trans[0]
                translation_object.y = trans[1]
                translation_object.z = trans[2]
                haro_world_tf_translation.publish(translation_object)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
    
            
    
            rate.sleep()