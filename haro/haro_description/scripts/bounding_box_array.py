#!/usr/bin/env python
import rospy
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
import threading
import math
from geometry_msgs.msg import Pose, Vector3
import tf
import std_msgs.msg
import random
"""
[jsk_recognition_msgs/BoundingBoxArray]:                                                                                       
std_msgs/Header header                                                                                                         
  uint32 seq                                                                                                                   
  time stamp                                                                                                                   
  string frame_id                                                                                                              
jsk_recognition_msgs/BoundingBox[] boxes
"""

class BoundingBoxArrayPublisher(object):
    def __init__(self):
        self.pub = rospy.Publisher('/dummy_bounding_box_array', BoundingBoxArray, queue_size=1)
        self.init_boundingboxarray()
        self.state = 0

    
    def get_init_time(self):
        self.init_seconds = rospy.get_time()
        
            
    def publish_once(self, boundingBoxArray_object):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(boundingBoxArray_object)
                break
            else:
                rospy.loginfo("Waiting for Subscriber...")
                rate.sleep()

    def init_boundingboxarray(self, num_boxes=30):
        self.boundingBoxArray_object = BoundingBoxArray()
        
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        h.frame_id = "world"
        
        self.boundingBoxArray_object.header = h
        
        self.minimum_dimension = 0.2
        self.init_x_position = 1.0

        for i in range(num_boxes):
            new_box = BoundingBox()
            new_box.header = h
            
            new_box.pose = Pose()
            new_box.pose.position.x = self.init_x_position + i*self.minimum_dimension
            
            new_box.dimensions = Vector3()
            new_box.dimensions.x = self.minimum_dimension
            new_box.dimensions.y = self.minimum_dimension
            new_box.dimensions.z = self.minimum_dimension
            
            new_box.label = i
            new_box.value = i*self.minimum_dimension

            
            self.boundingBoxArray_object.boxes.append(new_box)
        
        self.publish_once(self.boundingBoxArray_object)
        
        
        
    def start_bounding_box_array_demo(self):

        
        rate = rospy.Rate(5)
        self.get_init_time()
        i = 0
        while not rospy.is_shutdown():
            now = rospy.get_time()
            dif_seconds = now - self.init_seconds
            if i == 20:
                self.state += 1
                i = 0
                if self.state > 3:
                    self.state = 0
                else:
                    pass
            else:
                pass

            for index, box in enumerate(self.boundingBoxArray_object.boxes):
                index_value = index/10.0
                angle = dif_seconds + index_value
                
                if self.state == 0:
                    self.boundingBoxArray_object.boxes[index].pose.position.x = self.init_x_position + index*self.minimum_dimension
                    self.boundingBoxArray_object.boxes[index].pose.position.y = math.sin(angle)
                    self.boundingBoxArray_object.boxes[index].pose.position.z = 0.0
                elif self.state == 1:
                    self.boundingBoxArray_object.boxes[index].pose.position.x = self.init_x_position + index*self.minimum_dimension
                    self.boundingBoxArray_object.boxes[index].pose.position.y = 0.0
                    self.boundingBoxArray_object.boxes[index].pose.position.z = math.sin(angle)
                elif self.state == 2:
                    self.boundingBoxArray_object.boxes[index].pose.position.x = self.init_x_position + index*self.minimum_dimension
                    self.boundingBoxArray_object.boxes[index].pose.position.y = math.sin(angle)
                    self.boundingBoxArray_object.boxes[index].pose.position.z = math.cos(angle)
                else:
                    self.boundingBoxArray_object.boxes[index].pose.position.x = math.sin(angle)
                    self.boundingBoxArray_object.boxes[index].pose.position.y = math.cos(angle)
                    self.boundingBoxArray_object.boxes[index].pose.position.z = 0.0
                
                
            
            self.publish_once(self.boundingBoxArray_object)
            rate.sleep()
            
            i += 1
            



def start_demo():
    rospy.init_node('dummyBoundingBoxArrayPublisher_node', anonymous=True)
    boundingbox_array_object = BoundingBoxArrayPublisher()
    try:
        boundingbox_array_object.start_bounding_box_array_demo()
    except rospy.ROSInterruptException:
        boundingbox_array_object.close()

if __name__ == '__main__':
    start_demo()