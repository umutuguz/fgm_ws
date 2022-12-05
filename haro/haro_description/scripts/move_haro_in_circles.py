#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class HaroMove():
    
    def __init__(self):
        self.haro_vel = rospy.Publisher('/haro/cmd_vel', Twist, queue_size=1)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10) # 10hz
    
    def shutdownhook(self):
            # works better than the rospy.is_shut_down()
            print("shutdown time! Stop the robot")
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.haro_vel.publish(cmd)
            self.ctrl_c = True

    def start(self):
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 1.0
        while not self.ctrl_c:
            self.haro_vel.publish(cmd)
            self.rate.sleep()
    
    def move_for_x_time(self, num=20):
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 1.0
        i = 0
        while i <= num:
            rospy.loginfo(i)
            self.haro_vel.publish(cmd)
            self.rate.sleep()
            i += 1
        self.shutdownhook()
            
    def move_for_x_time_updown(self, num=20):
        cmd = Twist()
        i = 0
        while i <= num:
            if i > num/2.0:
                cmd.linear.z = -0.5
                rospy.loginfo("Move Down")
            else:
                cmd.linear.z = 0.5
                rospy.loginfo("Move Up")
            self.haro_vel.publish(cmd)
            self.rate.sleep()
            i += 1
        self.shutdownhook()

if __name__ == '__main__':
    rospy.init_node('move_haro_in_circles_node', anonymous=True)
    haro_object = HaroMove()
    try:
        haro_object.move_for_x_time(num=20)
    except rospy.ROSInterruptException:
        pass