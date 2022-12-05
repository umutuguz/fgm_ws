#!/usr/bin/env python

# it depends on jsk_rviz_plugins
import random
import rospy
from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
from std_msgs.msg import ColorRGBA
# Example 
# roslaunch jsk_rviz_plugins overlay_sample.launch
class OverlayTextPublisher(object):
    def __init__(self):
        rospy.init_node("static_overlay_text")

    def publish_text(self,event):
        self.text_interface.publish(str(random.randint(0,100)))
    
    
    def simple_random_text_overlaytextinterface_demo(self):
        self.text_interface = OverlayTextInterface("~output") 
        rospy.Timer(rospy.Duration(0.1), self.publish_text)
        rospy.spin()
        
    def only_overlaytext(self):
        rate = rospy.Rate(1)
        pub = rospy.Publisher("/random_text", OverlayText, queue_size=1)
        i = 0
        while not rospy.is_shutdown():
            
            msg = OverlayText()
            msg.width = 100
            msg.height = 100
            msg.left = 10 + i*10
            msg.top = 10
            msg.text_size = 20
            msg.line_width = 2
            msg.font = "DejaVu Sans Mono"
            msg.text = str(random.randint(0,100))
            # Per One 0-255 RGBA
            #msg.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
            msg.fg_color = ColorRGBA(random.random(), random.random(), random.random(), 1.0)
            msg.bg_color = ColorRGBA(random.random(), random.random(), random.random(), random.random())
            pub.publish(msg)
            rate.sleep()
            i += 1

    
if __name__ == "__main__":
    overlay_object = OverlayTextPublisher()
    #overlay_object.simple_random_text_overlaytextinterface_demo()
    overlay_object.only_overlaytext()
    