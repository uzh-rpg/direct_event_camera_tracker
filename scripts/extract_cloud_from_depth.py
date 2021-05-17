#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tqdm import tqdm
import os
import argparse
from datetime import datetime

import cv2
import numpy as np
from scipy.interpolate import RectBivariateSpline
from pyquaternion import Quaternion

from plyfile import PlyData, PlyElement
import yaml
import xml.etree.ElementTree as ET
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

"""
    v1 ---- v2
     |    / |
     |   /  |
     |  /   |
     | /    |
    v3----- v4
"""
def build_faces(v1, v2, v3, v4):
    if v2 < 0 or v3 < 0:
        return []

    f = []

    if v1 >= 0:
        # build upper left triangle
        f.append( ([v1, v3, v2],) )

    if v4 >= 0:
        # build lower right triangle
        f.append( ([v4, v2, v3],) )

    return f


def convert_to_cloud(color, color_intrinsics, depth, depth_intrinsics, R_ev_depth, t_ev_depth, make_mesh=False):

    if color_intrinsics['camera_name'].startswith('DAVIS'):
        use_intel_color = False
    else:
        use_intel_color = True

    K_ev = np.array(color_intrinsics['camera_matrix']['data']).reshape((3,3))
    K_de = np.array(depth_intrinsics['camera_matrix']['data']).reshape((3,3))

    depth_fx = K_de[0,0]
    depth_fy = K_de[1,1]
    depth_cx = K_de[0,2]
    depth_cy = K_de[1,2]

    col_fx   = K_ev[0,0]
    col_fy   = K_ev[1,1]
    col_cx   = K_ev[0,2]
    col_cy   = K_ev[1,2]

    if use_intel_color:
        assert (K_de == K_ev).all()

    col_dist = np.array(color_intrinsics['distortion_coefficients']['data'])

    vertex_dtype = [
            ('x',     np.float32),
            ('y',     np.float32),
            ('z',     np.float32),
            ('red',   np.uint8),
            ('green', np.uint8),
            ('blue',  np.uint8)
            ]

    if use_intel_color:
        interpolator_R = RectBivariateSpline(range(color.shape[0]), range(color.shape[1]), color[:,:,0])
        interpolator_G = RectBivariateSpline(range(color.shape[0]), range(color.shape[1]), color[:,:,1])
        interpolator_B = RectBivariateSpline(range(color.shape[0]), range(color.shape[1]), color[:,:,2])
    else:
        interpolator = RectBivariateSpline(range(color.shape[0]), range(color.shape[1]), color)

    points = []
    depths = []
    vertex_indexes = np.zeros(shape=depth.shape, dtype=int)
    for y in tqdm(range(depth.shape[0])):
        for x in range(depth.shape[1]):

            # mark index as invalid
            vertex_indexes[y,x] = -1

            # values are in mm -> convert to meter
            Z = depth[y,x] / 1000.0

            # ignore points without depth
            if Z <= 0:
                continue

            depths.append(Z) # just for statistical purposes

            # back-project pixel into 3D point
            P_depth = np.array([
                    [(x-depth_cx)/depth_fx * Z],
                    [(y-depth_cy)/depth_fy * Z],
                    [Z],
                ])


            # transform point from depth camera frame to event camera frame
            #if use_intel_color:
                #P_ev = P_depth + np.array([[-0.0588],[0],[0]])
            #else:
            P_ev = R_ev_depth.rotate(P_depth) + t_ev_depth     # use quaternion + translation vector
                #P_ev = T_HE[0:3,0:3].dot(P_depth) + T_HE[0:3,3]  # use T_HE

            # project onto color image to sample intensity
            px_ev = cv2.projectPoints(np.array([P_ev]), (0,0,0), (0,0,0), K_ev, col_dist)[0][0][0]

            if px_ev[0] < 0 or px_ev[1] < 0 or px_ev[0] > color.shape[1]-2 or px_ev[1] > color.shape[0]-2:
                c = 1 # make invalid pixels visible for debugging
                continue # skip pixels that fall outside of color image
            else:
                if use_intel_color:
                    cr = interpolator_R.ev(px_ev[1], px_ev[0])
                    cg = interpolator_G.ev(px_ev[1], px_ev[0])
                    cb = interpolator_B.ev(px_ev[1], px_ev[0])
                else:
                    c = interpolator.ev(px_ev[1], px_ev[0]) # (row, col) ordering

                    cr = c
                    cg = c
                    cb = c

            if cr < 0:
                cr = 0
            if cg < 0:
                cg = 0
            if cb < 0:
                cb = 0


            #c = color[int(px_ev[1]+0.5), int(px_ev[0]+0.5)] * 2

            vertex_indexes[y,x] = len(points)
            points.append(
                np.array((
                    P_ev[0], P_ev[1], P_ev[2],
                    cr, cg, cb),
                    dtype = vertex_dtype
                ))

    verts = np.array(points, dtype=vertex_dtype)

    depths = np.array(depths)
    print "average scene depth: ", np.average(depths), "m"
    print "min depth: ", np.min(depths), "m"
    print "max depth: ", np.max(depths), "m"

    faces = []
    if make_mesh:
        print "building mesh..."
        for y in range(depth.shape[0]-1):
            for x in range(depth.shape[1]-1):
                faces += build_faces(
                    vertex_indexes[y,x],
                    vertex_indexes[y,x+1],
                    vertex_indexes[y+1,x],
                    vertex_indexes[y+1,x+1])

        faces = np.array(faces, dtype=[('vertex_indices', 'i4', (3,))])

    print "appending", len(verts), "points"
    el = [PlyElement.describe(verts, 'vertex')]

    if make_mesh:
        print "appending", len(faces), "faces"
        el.append(PlyElement.describe(faces, 'face'))

    return PlyData(el)


def get_first_msg(bag, topic):
    for topic, msg, t in bag.read_messages(topics=[topic]):
        return msg

    print "Rosbag does not contain any messages on topic '" + str(topic) + "'"
    raise "invalid input"


def main():
    parser = argparse.ArgumentParser(description="Extract depth and color images from a rosbag and convert them to a pointcloud")
    parser.add_argument("--bag", help="Input ROS bag.", required=True)
    parser.add_argument("--output", help="Output path for cloud.")
    parser.add_argument("--image_topic", default="/dvs/", help="Image topic.")
    parser.add_argument("--depth_topic", default="/camera/depth/", help="Depth topic.")
    parser.add_argument("--image_seq", type=int, help="sequence number of color image to extract")
    parser.add_argument("--depth_seq", type=int, help="sequence number of depth to extract")
    parser.add_argument("--image_calib", help="calibration file for intensity (yaml)", required=True)
    parser.add_argument("--depth_calib", help="calibration file of depth camera (yaml)", required=True)
    parser.add_argument("--depth_ev_transf", help="launch file containing T_depth_ev", required=True)
    parser.add_argument("--make_mesh", action="store_true", help="build a mesh instead of a cloud")

    args = parser.parse_args()

    if not args.output:
        args.output = os.path.splitext(args.bag)[0] + '.ply'

    # load calibration files
    with open(args.image_calib, 'r') as f:
        img_calib = yaml.load(f)

    with open(args.depth_calib, 'r') as f:
        depth_calib = yaml.load(f)

    # load transformation
    transf_cfg = ET.parse(args.depth_ev_transf).getroot()
    n = transf_cfg.find("./node[@name='davis_vo']")

    R_depth_ev = Quaternion(
        x=float(n.find("./param[@name='T_depth_ev/qx']").attrib['value']),
        y=float(n.find("./param[@name='T_depth_ev/qy']").attrib['value']),
        z=float(n.find("./param[@name='T_depth_ev/qz']").attrib['value']),
        w=float(n.find("./param[@name='T_depth_ev/qw']").attrib['value'])
        )

    t_depth_ev = np.array([
        float(n.find("./param[@name='T_depth_ev/tx']").attrib['value']),
        float(n.find("./param[@name='T_depth_ev/ty']").attrib['value']),
        float(n.find("./param[@name='T_depth_ev/tz']").attrib['value'])
    ])

    print "R_depth_ev:", R_depth_ev
    print "t_depth_ev:", t_depth_ev


    # invert transformation
    R_ev_depth = R_depth_ev.inverse
    t_ev_depth = - R_ev_depth.rotate(t_depth_ev)

    bridge = CvBridge()

    bag = rosbag.Bag(args.bag, "r")

    col_img = None
    col_t = None
    for topic, msg, t in bag.read_messages(topics=[args.image_topic+"image_raw"]):
        if not args.image_seq or args.image_seq == msg.header.seq:
            col_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            col_t = t
            #print "using intensity image from t =", t
            break

    if col_img is None:
        print "ERROR: No image found on topic '" + args.image_topic+"image_raw'"
        return

    depth = None
    for topic, msg, t in bag.read_messages(topics=[args.depth_topic+"image_raw"]):
        if not args.depth_seq or args.depth_seq == msg.header.seq:
            depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            #print "using depth map from t =", t
            print "time difference between intensity image and depth map:", (col_t - t).to_sec()*1000.0, "ms"
            break

    mesh = convert_to_cloud(
            col_img, img_calib,
            depth,   depth_calib,
            R_ev_depth, t_ev_depth,
            args.make_mesh)

    # put all the information that was used to run this script into header for later reference
    mesh.comments += ['this model was created on ' + str(datetime.now()) + ' by extract_cloud_from_depth.py']

    for arg in vars(args):
        a = getattr(args, arg)
        if arg in ["bag", "output", "depth_calib", "image_calib", "depth_ev_transf"]:
            a = os.path.abspath(a)
        else:
            a = str(a)

        mesh.comments += ["parameter: " + str(arg) + " = " + a]

    mesh.write(args.output)

    bag.close()



if __name__ == '__main__':
    main()
