#!/usr/bin/env python

import rospy
import os
import time
import re
import signal
from move_base_msgs.msg import MoveBaseActionGoal

os.system("roscore")

time.sleep(2)

os.system("killall rosmaster")