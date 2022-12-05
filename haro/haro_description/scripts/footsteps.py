#!/usr/bin/env python
import rospy
from jsk_footstep_msgs.msg import Footstep, FootstepArray
import math
from geometry_msgs.msg import Pose, Vector3
import tf
import std_msgs.msg
import random

"""
[jsk_footstep_msgs/FootstepArray]:
std_msgs/Header header                                                                                                         
  uint32 seq                                                                                                                   
  time stamp                                                                                                                   
  string frame_id                                                                                                              
jsk_footstep_msgs/Footstep[] footsteps

[jsk_footstep_msgs/Footstep]:                                                                                                                                             
uint8 RIGHT=2
uint8 LEFT=1

# Constants to visualize progress --> This doesnt change anything, it might be some legacy code
uint8 REJECTED=3
uint8 APPROVED=4

## limb_indicator values
uint8 LLEG=1
uint8 RLEG=2
uint8 LARM=5
uint8 RARM=6

uint8 leg ## value should be one of limb_indicator values.
geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.
duration duration
# optional parameters
uint32 footstep_group
geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]
geometry_msgs/Vector3 offset     ## offset from pose to center of cube
float32 swing_height
float32 cost
"""


class FootStepArrayPublisher(object):
    def __init__(self):
        self.pub = rospy.Publisher('/dummy_footstep_array', FootstepArray, queue_size=1)
        self.init_footstep_array()
        self.state = 0

    
    def get_init_time(self):
        self.init_seconds = rospy.get_time()
        
            
    def publish_once(self, object_to_publish):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(object_to_publish)
                break
            else:
                rospy.loginfo("Waiting for Subscriber...")
                rate.sleep()


    def generate_new_footstep(self, pos_x=0, pos_y=0, yaw=0, is_right=True, is_leg=True, group_num = 1, regected=False, approved=False):
        footstep_object = Footstep()
    
        if is_leg:
            if is_right:
                footstep_object.leg = footstep_object.RLEG
                
                footstep_object.pose.position.x = pos_x / 2.0
                footstep_object.pose.position.y = pos_y / 2.0
            else:
                footstep_object.leg = footstep_object.LLEG
                
                footstep_object.pose.position.x = pos_x
                footstep_object.pose.position.y = pos_y
            
            footstep_object.pose.position.z = 0.0
            
        else:
            if is_right:
                footstep_object.leg = footstep_object.RARM
                
                footstep_object.pose.position.x = pos_x / 2.0
                footstep_object.pose.position.y = pos_y / 2.0
                
            else:
                footstep_object.leg = footstep_object.LARM
                
                footstep_object.pose.position.x = pos_x
                footstep_object.pose.position.y = pos_y
            
            footstep_object.pose.position.z = 1.5
            
        if regected:
            footstep_object.leg = footstep_object.REJECTED
        if approved:
            footstep_object.leg = footstep_object.APPROVED
        
        
        
        
        # ai, aj, ak == roll, pitch, yaw
        quaternion = tf.transformations.quaternion_from_euler(ai=0, aj=0, ak=yaw)
        footstep_object.pose.orientation.x = quaternion[0]
        footstep_object.pose.orientation.y = quaternion[1]
        footstep_object.pose.orientation.z = quaternion[2]
        footstep_object.pose.orientation.w = quaternion[3]
        
        
        footstep_object.duration = rospy.Duration.from_sec(0.2)
        
        # Optional Values
        
        footstep_object.footstep_group = group_num
        """
        footstep_object.dimensions_object = Vector3()
        footstep_object.offset_object = Vector3()
        
        """
        # Not shown in RVIZ, for locomotion purposes only
        footstep_object.swing_height = 2.0
        
        footstep_object.cost = 3.25
        
        return footstep_object

    def init_footstep_array(self):
        self.footstepArray_object = FootstepArray()
        new_footstep = self.generate_new_footstep()
        
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        h.frame_id = "world"
        self.footstepArray_object.header = h

        self.footstepArray_object.footsteps.append(new_footstep)
        
        self.publish_once(self.footstepArray_object)
        
        
        
    def start_demo(self):

        
        rate = rospy.Rate(1)
        self.get_init_time()
        i = 0
        
        is_right = (i % 2 == 0)
        #is_leg = (i % 3 == 0)
        is_leg = True
        
        max_steps = 10
        
        while not rospy.is_shutdown():
            angle = i/5.0
            
            is_right = (i % 2 == 0)
            #is_leg = (i % 3 == 0)
            is_leg = True
            
            group_num = i % 2
            
            new_footstep = self.generate_new_footstep(  pos_x=math.sin(angle),
                                                        pos_y=math.cos(angle),
                                                        yaw=-angle,
                                                        is_right=is_right,
                                                        is_leg = is_leg,
                                                        group_num = group_num)
            
            # It will appear the path in the order you append, no matter if you append once at a time or a lot
            self.footstepArray_object.footsteps.append(new_footstep)
            
            if len(self.footstepArray_object.footsteps) > max_steps:
                self.footstepArray_object.footsteps.pop(0)
            
            i += 1
            self.publish_once(self.footstepArray_object)
            rate.sleep()
            
            



def main_start_demo():
    rospy.init_node('dummyFootStepArrayPublisher_node', anonymous=True)
    footstep_array_object = FootStepArrayPublisher()
    try:
        footstep_array_object.start_demo()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main_start_demo()