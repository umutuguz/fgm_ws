#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.srv import EusCommand, EusCommandRequest, EusCommandResponse 
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from move_haro_in_circles import HaroMove

class RobotCommandServer(object):
    def __init__(self):
        # To make the robot command RVIZ panel work it has to be this name
        service_server_name = "/eus_command"
        s = rospy.Service(service_server_name, EusCommand, self.handle_command)
        rospy.loginfo("Service =>"+str(service_server_name)+",Ready to recieve commands...")
        
        updown_service_server_name = "/move_up_and_down"
        s = rospy.Service(updown_service_server_name, Empty, self.handle_updown_command)
        rospy.loginfo("Service =>"+str(updown_service_server_name)+",Ready to recieve commands...")
        
        self.haro_object = HaroMove()
        

    def handle_command(self,req):
        """
        [jsk_rviz_plugins/EusCommand.srv]
        string command                                                                                                                 
        ---  
        """
        rospy.loginfo("Command Requested==>"+str(req.command))
        if req.command == "move_in_circles":
            self.haro_object.move_for_x_time(num=20)
        return EusCommandResponse()
        
    def handle_updown_command(self,req):
        """
        [std_srvs/Empty.srv]
        ---  
        """
        rospy.loginfo("Move Up And Down")
        self.haro_object.move_for_x_time_updown(num=100)
        return EmptyResponse()
    
    def start_server_spin(self):
        rospy.spin()



if __name__ == "__main__":
    rospy.init_node('robot_command_server')
    robotserver_object = RobotCommandServer()
    robotserver_object.start_server_spin()