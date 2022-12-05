#!/usr/bin/env python

import sys
import rospy
from jsk_gui_msgs.srv import YesNo, YesNoRequest, YesNoResponse


class RvizPanels(object):
    def __init__(self):
        self.yes_or_no_service_name = '/rviz/yes_no_button'
        rospy.wait_for_service(self.yes_or_no_service_name)
        rospy.loginfo(self.yes_or_no_service_name+"...Ready")
        
    def ask_yes_or_no_rviz(self):
    
        try:
            ask_yes_or_no_proxy = rospy.ServiceProxy(self.yes_or_no_service_name, YesNo)
            yes_no_req = YesNoRequest()
            responce = ask_yes_or_no_proxy(yes_no_req)
            return responce.yes
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


        

if __name__ == "__main__":
    rospy.init_node('demorvizpanels_node', anonymous=True)
    rvizpanels_object = RvizPanels()
    rospy.loginfo("Waiting for Answer in RVIZ panel")
    yes_answer = rvizpanels_object.ask_yes_or_no_rviz()
    rospy.loginfo("Answer Yes="+str(yes_answer))
