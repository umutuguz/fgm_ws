#!/usr/bin/env python
"""
Created By: Miguel Angel Rodriguez in TheConstruct
This allows things like doing "Hokiti Pokiti" in Disney Film Merlin
cleaning up whole automatic learning setups, specially the reset buttons.
"""
import rospy
import time
from geometry_msgs.msg import Vector3
from apply_forces_objects_tools_pkg.apply_wrench import apply_force_robot


class HaroActuator(object):
    def __init__(self):
        print("Init Taskboard Actuator")
        self.haro_body_link_name = "haro::haro_base_link"

    def init_ros_node(self):
        rospy.init_node('haro_systems_actuator', anonymous=True)

    def move_haro(self, force_magnitud):
        link_name = self.haro_body_link_name
        time_apply_force = 0.1
        
        
        rospy.loginfo("Config Apply Force")
        
        force = Vector3()
        torque = Vector3()
        
        force.x = 0.0
        force.y = 0.0
        force.z = force_magnitud
        
        torque.x = 0.0
        torque.y = 0.0
        torque.z = 0.0
        
        apply_force_robot(force, torque, body_name=link_name)


    def start(self):
        rospy.loginfo("F=1")
        self.move_haro(force_magnitud=-1)
        time.sleep(1)
        rospy.loginfo("F=0")
        self.move_haro(force_magnitud=0)
        rospy.spin()


if __name__ == '__main__':
    haro_actuator = HaroActuator()
    haro_actuator.init_ros_node()
    try:
        haro_actuator.start()
    except rospy.ROSInterruptException:
        pass
