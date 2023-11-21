#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from UrDashboardRob import UrDashboardRobotnik



def main():

    rospy.init_node("UrDashboardRob_node")

    rc_node = UrDashboardRobotnik()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
