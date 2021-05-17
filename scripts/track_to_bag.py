#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tqdm import tqdm
import os
import argparse
import csv

import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a trajectory (CSV) into a rosbag that can be played back.")
    parser.add_argument("track", help="Input trajectory file.")
    parser.add_argument("--output", help="Output path for rosbag.")
    parser.add_argument("--images", help="Folder with images that will be added to rosbag.")
    args = parser.parse_args()

    if not args.output:
        args.output = args.track + ".bag"

    bag = rosbag.Bag(args.output, 'w')
    bridge = CvBridge()


    image_folders = []
    if args.images:

        for f in next(os.walk(args.images))[1]:
            image_folders.append(os.path.abspath(os.path.join(args.images, f)))

        print "found the following image folders:"
        for f in image_folders:
            print "  -", os.path.basename(f)


    with open(args.track, 'r') as csvfile:
        total_rows = sum(1 for line in csvfile)-1

    with open(args.track, 'r') as csvfile:
        seq = 0
        for row in tqdm(csv.DictReader(csvfile), total=total_rows):
            pose = PoseStamped()

            stamp = row['time'].split('.')
            pose.header.stamp = rospy.Time(int(stamp[0]), int( "{:<09}".format(stamp[1]) )) # make sure 0.5 sec isn't interpreted as 5ns
            pose.header.frame_id = "map"
            pose.header.seq = seq
            seq += 1

            pose.pose.position.x = float(row['Position X'])
            pose.pose.position.y = float(row['Position Y'])
            pose.pose.position.z = float(row['Position Z'])

            pose.pose.orientation.x = float(row['Rotation X'])
            pose.pose.orientation.y = float(row['Rotation Y'])
            pose.pose.orientation.z = float(row['Rotation Z'])
            pose.pose.orientation.w = float(row['Rotation W'])

            bag.write("/track", pose, pose.header.stamp)


            # write same pose as tf



            for folder in image_folders:
                filename = folder + "/{}.png".format(row["step"])
                try:
                    img = cv2.imread(filename)
                    if img is None:
                        raise Exception("Image is None")
                except Exception as e:
                    print "Failed to read image '" + filename + "':"
                    print e
                    print "Ignoring folder '" + folder + "' from now on."
                    image_folders.remove(folder)
                    continue

                img_msg = bridge.cv2_to_imgmsg(img)

                img_msg.header = pose.header

                bag.write("/images/" + os.path.basename(folder), img_msg, img_msg.header.stamp)



    bag.close()
