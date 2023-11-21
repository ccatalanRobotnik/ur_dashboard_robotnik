#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dashboard.dashboard import DashboardHelper
from rcomponent.rcomponent import *

# Insert here general imports:
import math
import rospy
# Insert here msg and srv imports:
from std_msgs.msg import String
from robotnik_msgs.msg import StringStamped

from std_srvs.srv import Trigger, TriggerResponse
from robotnik_msgs.srv import SetString, SetStringResponse


class UrDashboardRobotnik(RComponent):
    """
    alternative dashboard for UR robots
    """

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.UR_IP = rospy.get_param('~robot_ip')

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Publisher
        self.status_pub = rospy.Publisher('~status', String, queue_size=10)
        self.status_stamped_pub = rospy.Publisher('~status_stamped', StringStamped, queue_size=10)
        #Services
        #self.ur_dashboard_read = rospy.Service('~read_string', Trigger, self.ur_dashboard_read_cb)
        self.ur_dashboard_quit = rospy.Service('~quit', Trigger, self.ur_dashboard_quit_cb)
        self.ur_dashboard_connect = rospy.Service('~connect', Trigger, self.ur_dashboard_connect_cb)
        self.ur_dashboard_play = rospy.Service('~play', Trigger, self.ur_dashboard_play_cb)
        self.ur_dashboard_stop = rospy.Service('~stop', Trigger, self.ur_dashboard_stop_cb)
        self.ur_dashboard_pause = rospy.Service('~pause', Trigger, self.ur_dashboard_pause_cb)
        self.ur_dashboard_load_program = rospy.Service('~load_program', SetString, self.ur_dashboard_load_program_cb)

        return 0

    def init_state(self):
        self.status = String()
        self.ur_dashboard = DashboardHelper(self.UR_IP)
        
        RComponent.setup(self)
        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Check topic health

        if(self.check_topics_health() == False):
            self.switch_to_state(State.EMERGENCY_STATE)
            return RComponent.ready_state(self)

        # Publish topic with status

        status_stamped = StringStamped()
        status_stamped.header.stamp = rospy.Time.now()
        status_stamped.string = self.status.data

        self.status_pub.publish(self.status)
        self.status_stamped_pub.publish(status_stamped)

        return RComponent.ready_state(self)

    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)


    def ur_dashboard_read_cb(self, req):
        rospy.loginfo("Reading dashboard")
        response = TriggerResponse()
        response.message = self.ur_dashboard.get_reply()
        return response


    def ur_dashboard_quit_cb(self, req):
        rospy.loginfo("Disconnecting dashboard")
        response = TriggerResponse()
        response = self.ur_dashboard.quit()
        return response

    def ur_dashboard_connect_cb(self, req):
        rospy.loginfo("Connecting dashboard")
        response = TriggerResponse()
        response = self.ur_dashboard.connect()
        return response


    def ur_dashboard_play_cb(self, req):
        rospy.loginfo("Sending play")
        response = TriggerResponse()
        response = self.ur_dashboard.play()
        return response


    def ur_dashboard_stop_cb(self, req):
        rospy.loginfo("Sending stop ")
        response = TriggerResponse()
        response = self.ur_dashboard.stop()
        return response


    def ur_dashboard_pause_cb(self, req):
        rospy.loginfo("Sending pause")
        response = TriggerResponse()
        response = self.ur_dashboard.pause()
        return response


    def ur_dashboard_pause_cb(self, req):
        rospy.loginfo("Sending pause")
        response = TriggerResponse()
        response = self.ur_dashboard.pause()
        return response


    def ur_dashboard_load_program_cb(self, req):
        rospy.loginfo("Loading program")
        response = SetStringResponse()
        response_Trig = self.ur_dashboard.load_program(req.data)
        response.ret.success = response_Trig.success
        response.ret.message = response_Trig.message
        return response