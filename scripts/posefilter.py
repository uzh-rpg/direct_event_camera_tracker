#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import argparse
import rospy
from visualization_msgs.msg import Marker
import numpy as np

from pyquaternion import *


from geometry_msgs.msg import PoseStamped


class PoseFilter:
    def __init__(self, args):
        self.args = args

        rospy.Subscriber(self.args.topic, PoseStamped, self.pose_callback)
        self.pub = rospy.Publisher('/filtered' + self.args.topic, PoseStamped, queue_size=0)

        self.buffer = []

    def pose_callback(self, msg):

        p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z])


        self.buffer.append(p)

        if len(self.buffer) > args.buflen:
            self.buffer = self.buffer[1:] # remove oldest

        if self.args.justdelay:
            filtered_pos = self.buffer[0]
        else:
            filtered_pos = np.median(self.buffer, axis=0)

        msg.pose.position.x = filtered_pos[0]
        msg.pose.position.y = filtered_pos[1]
        msg.pose.position.z = filtered_pos[2]

        self.pub.publish(msg)
        #print "pose", p, "filtered to", filtered_pos



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Filter poses with running average (median)")
    parser.add_argument("topic", help="topic with poses to filter")
    parser.add_argument("--buflen", type=int, default=20, help="size of filter")
    parser.add_argument("--justdelay", action="store_true", help="don't actually filter, just delay pose")
    args = parser.parse_args()

    rospy.init_node('pose_filter', anonymous=True)
    pfilter = PoseFilter(args)
    print "pose filter running, listening to", args.topic
    rospy.spin()
