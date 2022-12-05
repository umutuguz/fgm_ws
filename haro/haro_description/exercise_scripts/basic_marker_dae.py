#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MarkerBasics(object):

    def __init__(self):
        self.marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_marker(index=0,z_val=0)
    
    def init_marker(self,index=0, z_val=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "/world"
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = "haro"
        self.marker_object.id = index
        self.marker_object.type = Marker.MESH_RESOURCE
        self.marker_object.action = Marker.ADD
        
        my_point = Point()
        my_point.z = z_val
        self.marker_object.pose.position = my_point
        
        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0
        self.marker_object.scale.x = 2.0
        self.marker_object.scale.y = 2.0
        self.marker_object.scale.z = 2.0
    
        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 0.0
        # This has to be otherwise it will be transparent
        self.marker_object.color.a = 0.0
        self.marker_object.mesh_resource = "package://haro_description/meshes/turtle_real_textures.dae";
        self.marker_object.mesh_use_embedded_materials = True
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
    
