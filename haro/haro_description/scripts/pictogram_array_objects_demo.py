#!/usr/bin/env python

#
# Please run rviz by rosrun rviz rviz -d `rospack find jsk_rviz_plugins`/config/pictogram.rviz
#

import rospy
import math
from jsk_rviz_plugins.msg import Pictogram, PictogramArray
from random import random, choice
rospy.init_node("pictogram_object_demo_node")
p = rospy.Publisher("/pictogram_array", PictogramArray,  queue_size=1)

r = rospy.Rate(1)
actions = [Pictogram.JUMP, Pictogram.JUMP_ONCE, Pictogram.ADD, 
           Pictogram.ROTATE_X, Pictogram.ROTATE_Y, Pictogram.ROTATE_Z]
pictograms = ["fa-adn","fa-align-center","fa-align-justify"]
object_frames = ["/world","/person_standing","haro_base_link"]

while not rospy.is_shutdown():
    
    arr = PictogramArray()
    arr.header.frame_id = "/world"
    arr.header.stamp = rospy.Time.now()
    for index, character in enumerate(pictograms):
        msg = Pictogram()
        msg.header.frame_id = object_frames[index]
        #msg.action = actions[0]
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0.5
        # It has to be like this to have them verticaly oriented the icons.
        msg.pose.orientation.w = 0.7
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = -0.7
        msg.pose.orientation.z = 0
        msg.size = 1
        msg.color.r = 25 / 255.0
        msg.color.g = 255 / 255.0
        msg.color.b = 240 / 255.0
        msg.color.a = 1.0
        msg.character = character
        arr.pictograms.append(msg)
        
    p.publish(arr)
    r.sleep()

