#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import TwistStamped, Twist

class TwistStampedPublisher(object):
    def __init__(self):
        self.twist_stamped_publisher = rospy.Publisher('/haro/twist_stamped_publisher', TwistStamped, queue_size=1)
        rospy.Subscriber("/haro/cmd_vel", Twist, self.callback)
        
        self.current_twiststamped = TwistStamped()
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        h.frame_id = "haro_base_link"
        self.current_twiststamped.header = h
        
    def callback(self,msg):
        
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        h.frame_id = "haro_base_link"
        self.current_twiststamped.header = h
        self.current_twiststamped.twist = msg
        
        #rospy.loginfo(msg.linear)
        rospy.loginfo("TWIST STAMPED ===>"+str(self.current_twiststamped.twist.linear))
    
    def twiststamped_publisher(self):
        
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            
            self.twist_stamped_publisher.publish(self.current_twiststamped)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('twist_stamped_pub_node', anonymous=True)
    twiststamped_object = TwistStampedPublisher()
    try:
        twiststamped_object.twiststamped_publisher()
    except rospy.ROSInterruptException:
        pass