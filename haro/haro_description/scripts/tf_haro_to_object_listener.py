#!/usr/bin/env python
import sys
import rospy
import math
import tf
from geometry_msgs.msg import Twist, Vector3

if __name__ == '__main__':
    rospy.init_node('tf_listener_haro_to_person')

    listener = tf.TransformListener()

    if len(sys.argv) < 3:
        print("usage: tf_haro_to_object_listener.py parent child")
    else:
        follower_model_name = sys.argv[1]
        model_to_be_followed_name = sys.argv[2]
        
        topic_to_publish_name = "/"+str(follower_model_name)+"_to_"+str(model_to_be_followed_name)+"_tf_translation"
        # We will publish Twist message but its a positional data not speed, just reusing an existing structure.
        tf_translation = rospy.Publisher(topic_to_publish_name, Twist ,queue_size=1)
        
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
                
                angular = math.atan2(translation_object.y, translation_object.x)
                linear = math.sqrt(translation_object.x ** 2 + translation_object.y ** 2)
                cmd = Twist()
                cmd.linear.x = linear
                cmd.angular.z = angular
                tf_translation.publish(cmd)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
    
            
    
            rate.sleep()