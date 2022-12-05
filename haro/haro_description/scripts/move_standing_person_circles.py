#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class PersonMove():
    
    def __init__(self):
        self.person_vel = rospy.Publisher('/person_standing/cmd_vel', Twist, queue_size=1)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10) # 10hz
    
    def shutdownhook(self):
            # works better than the rospy.is_shut_down()
            print("shutdown time! Stop the robot")
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.person_vel.publish(cmd)
            self.ctrl_c = True

    def start(self):
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 0.5
        while not self.ctrl_c:
            self.person_vel.publish(cmd)
            self.rate.sleep()
    
    

if __name__ == '__main__':
    rospy.init_node('move_standing_person_circles_node', anonymous=True)
    person_object = PersonMove()
    try:
        person_object.start()
    except rospy.ROSInterruptException:
        pass