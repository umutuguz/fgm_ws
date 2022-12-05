#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
"""
[visualization_msgs/Marker]:                                                                                                                                              
uint8 ARROW=0                                                                                                                                                             
uint8 CUBE=1                                                                                                                                                              
uint8 SPHERE=2                                                                                                                                                            
uint8 CYLINDER=3                                                                                                                                                          
uint8 LINE_STRIP=4                                                                                                                                                        
uint8 LINE_LIST=5                                                                                                                                                         
uint8 CUBE_LIST=6                                                                                                                                                         
uint8 SPHERE_LIST=7                                                                                                                                                       
uint8 POINTS=8                                                                                                                                                            
uint8 TEXT_VIEW_FACING=9                                                                                                                                                  
uint8 MESH_RESOURCE=10                                                                                                                                                    
uint8 TRIANGLE_LIST=11                                                                                                                                                    
uint8 ADD=0                                                                                                                                                               
uint8 MODIFY=0                                                                                                                                                            
uint8 DELETE=2                                                                                                                                                            
std_msgs/Header header                                                                                                                                                    
  uint32 seq                                                                                                                                                              
  time stamp                                                                                                                                                              
  string frame_id                                                                                                                                                         
string ns                                                                                                                                                                 
int32 id                                                                                                                                                                  
int32 type                                                                                                                                                                
int32 action                                                                                                                                                              
geometry_msgs/Pose pose                                                                                                                                                   
  geometry_msgs/Point position                                                                                                                                            
    float64 x                                                                                                                                                             
    float64 y                                                                                                                                                             
    float64 z                                                                                                                                                             
  geometry_msgs/Quaternion orientation                                                                                                                                    
    float64 x                                                                                                                                                             
    float64 y                                                                                                                                                             
    float64 z                                                                                                                                                             
    float64 w                                                                                                                                                             
geometry_msgs/Vector3 scale                                                                                                                                               
  float64 x                                                                                                                                                               
  float64 y                                                                                                                                                               
  float64 z                                                                                                                                                               
std_msgs/ColorRGBA color                                                                                                                                                  
  float32 r                                                                                                                                                               
  float32 g                                                                                                                                                               
  float32 b 
duration lifetime                                                                                                                                                         
bool frame_locked                                                                                                                                                         
geometry_msgs/Point[] points                                                                                                                                              
  float64 x                                                                                                                                                               
  float64 y                                                                                                                                                               
  float64 z                                                                                                                                                               
std_msgs/ColorRGBA[] colors                                                                                                                                               
  float32 r                                                                                                                                                               
  float32 g                                                                                                                                                               
  float32 b                                                                                                                                                               
  float32 a                                                                                                                                                               
string text                                                                                                                                                               
string mesh_resource                                                                                                                                                      
bool mesh_use_embedded_materials
"""

class MarkerBasics(object):

    def __init__(self):
        self.marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_marker(index=0,z_val=0)
        self.init_marker(index=1,z_val=1)
    
    def init_marker(self,index=0, z_val=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "/world"
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = "haro"
        self.marker_object.id = index
        #self.marker_object.type = Marker.SPHERE
        self.marker_object.type = Marker.MESH_RESOURCE
        self.marker_object.action = Marker.ADD
        
        my_point = Point()
        my_point.z = z_val
        self.marker_object.pose.position = my_point
        
        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0
        self.marker_object.pose.orientation.w = 1.0
        self.marker_object.scale.x = 1.0
        self.marker_object.scale.y = 1.0
        self.marker_object.scale.z = 1.0
    
        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 0.0
        # This has to be otherwise it will be transparent
        self.marker_object.color.a = 1.0
        
        self.marker_object.mesh_resource = "package://haro_description/meshes/haro_gundam_green.dae";
    
        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)
    
    def start(self):
        while not rospy.is_shutdown():
            self.marker_objectlisher.publish(self.marker_object)
            self.rate.sleep()
   

if __name__ == '__main__':
    rospy.init_node('marker_basic_node', anonymous=True)
    markerbasics_object = MarkerBasics()
    try:
        markerbasics_object.start()
    except rospy.ROSInterruptException:
        pass
    
