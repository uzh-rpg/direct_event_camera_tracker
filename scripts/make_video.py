#!/usr/bin/env python

import os
import argparse
import csv

import rospy

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Build a video out of irregularly timed images")
    parser.add_argument("--timing", help="CSV with timestamps in seconds in second row (track.csv)", required=True)
    parser.add_argument("--output", default="ffmpeg_input.txt", help="output filename as input for ffmpeg")
    parser.add_argument("--scale", default=1, type=float, help="slowdown factor")

    args = parser.parse_args()

    last_stamp  = None
    with open(args.timing) as csvfile, open(args.output, 'w') as ffmpeg_input:

        ffmpeg_input.write("ffconcat version 1.0\n")

        for row in csv.DictReader(csvfile):
            stamp = row['time'].split('.')
            stamp = rospy.Time(int(stamp[0]), int(stamp[1]))


            if not last_stamp:
                last_stamp  = stamp
                continue

            ffmpeg_input.write("file '{}.png'\n".format(int(row['step'])-1))

            diff = (stamp - last_stamp).to_sec() * args.scale
            ffmpeg_input.write("duration {}\n".format(diff))


            last_stamp = stamp

        # repeat last frame (appearantly, ffmpeg wants this??)
        ffmpeg_input.write("file '{}.png'\n".format(int(row['step'])-1))

    print "now run:"
    print "ffmpeg -f concat -i ffmpeg_input.txt -vf fps=60 -c:v libx264 -crf 18 video.mp4"

