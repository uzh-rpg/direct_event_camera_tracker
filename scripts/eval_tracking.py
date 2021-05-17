#!/usr/bin/python3

import math
import os
import csv
import argparse
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion



###############################################################################
# PARAMETERS
###############################################################################

# in millimeters
figure_width  = 200
figure_height =  90


# matplotlib pgf details
###############################################################################

#plt.rcParams['figure.dpi'] = 300 # this doesn't seem to have any effect?

plt.rcParams['pgf.rcfonts'] = False # use fonts from LaTeX document
#plt.rcParams["text.usetex"] = True
#plt.rcParams["pgf.texsystem"] = "xelatex"
plt.rcParams['text.latex.unicode'] = True
#plt.rcParams["font.family"] = "serif"
#plt.rcParams["font.serif"]      = []
#plt.rcParams["font.sans-serif"] = []
#plt.rcParams["font.monospace"]  = []

###############################################################################
# UTILITIES
###############################################################################

CSV_INDEXES = [
        'step',
        'time',
        'Position X',
        'Position Y',
        'Position Z',
        'Rotation X',
        'Rotation Y',
        'Rotation Z',
        'Rotation W',
        'Lin. Velocity X',
        'Lin. Velocity Y',
        'Lin. Velocity Z',
        'Ang. Velocity X',
        'Ang. Velocity Y',
        'Ang. Velocity Z'
    ]

CSV_INDEXES_CAT = {
        'Position':      CSV_INDEXES.index('Position X'),
        'Rotation':      CSV_INDEXES.index('Rotation X'),
        'Lin. Velocity': CSV_INDEXES.index('Lin. Velocity X'),
        'Ang. Velocity': CSV_INDEXES.index('Ang. Velocity X'),
}


###############################################################################

def to_quaternion_arr(arr):
    return np.apply_along_axis(
        lambda r: Quaternion(r[[3,0,1,2]]).normalised,
        axis=1, arr=arr)

###############################################################################

# returns Tait-Bryan angles
# source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
def quaternion_to_euler_angle(quat):

    w = quat.real
    x = quat.imaginary[0]
    y = quat.imaginary[1]
    z = quat.imaginary[2]

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return np.array([X, Y, Z])

###############################################################################

# returns the angle between two vectors in degrees
# https://stackoverflow.com/a/13849249
def vector_angle(V):
    v1 = V[0:3]
    v2 = V[3:]

    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    return math.degrees(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

###############################################################################

def interpolate_gt(track_t, gt_t, gt_data):
    gt_poses_at_track_x = np.interp(track_t, gt_t, gt_data[:, 0])
    gt_poses_at_track_y = np.interp(track_t, gt_t, gt_data[:, 1])
    gt_poses_at_track_z = np.interp(track_t, gt_t, gt_data[:, 2])

    return np.column_stack((
        gt_poses_at_track_x,
        gt_poses_at_track_y,
        gt_poses_at_track_z))

###############################################################################
# PLOTTING
###############################################################################

def plot_xyz(values, gt=None):
    plt.plot(values[:,0], values[:,1], label='X tracked', color='red')
    plt.plot(values[:,0], values[:,2], label='Y tracked', color='green')
    plt.plot(values[:,0], values[:,3], label='Z tracked', color='blue')

    if gt is not None:
        plt.plot(gt[:,0], gt[:,1], label='X groundtruth', color='red',   linestyle='--', linewidth=3, alpha=0.3)
        plt.plot(gt[:,0], gt[:,2], label='Y groundtruth', color='green', linestyle='--', linewidth=3, alpha=0.3)
        plt.plot(gt[:,0], gt[:,3], label='Z groundtruth', color='blue',  linestyle='--', linewidth=3, alpha=0.3)

###############################################################################

def plot_values(track, gt, category):
    idx = CSV_INDEXES_CAT[category]

    # cut out interesting part of whole dataset
    track = track[:, [1, idx, idx+1, idx+2]]

    if gt is not None:
        gt = gt  [:, [0, idx-1, idx, idx+1]] # ground truth data doesn't have colum 'step'

    """
    if autoscale:
        n1 = np.linalg.norm(track[:, 1:], axis=1)
        n2 = np.linalg.norm(gt[:, 1:],    axis=1)
        scale = np.average(n2) / np.average(n1)
        print("scale:", scale)

        track[:, 1:] *= scale
        """

    plot_xyz(track, gt)

    plt.title(category)


###############################################################################

def plot_quaternions(track, gt):
    idx = CSV_INDEXES_CAT['Rotation']

    track_quats = to_quaternion_arr(track[:, idx:idx+4])
    gt_quats    = to_quaternion_arr(gt   [:, idx-1:idx+3])


    track_angles = np.array(list(map(quaternion_to_euler_angle, track_quats)))
    gt_angles    = np.array(list(map(quaternion_to_euler_angle, gt_quats)))

    plot_xyz(
            np.concatenate((np.reshape(track[:,1], (-1,1)), track_angles), axis=1),
            np.concatenate((np.reshape(gt[:,0],    (-1,1)), gt_angles), axis=1))

    plt.title('Rotation (Tait-Bryan angles)')

    # TODO: use proper quaternion interpolation instead of linear one
    gt_interpolated_x = np.interp(track[:,1], gt[:,0], gt[:,idx-1])
    gt_interpolated_y = np.interp(track[:,1], gt[:,0], gt[:,idx  ])
    gt_interpolated_z = np.interp(track[:,1], gt[:,0], gt[:,idx+1])
    gt_interpolated_w = np.interp(track[:,1], gt[:,0], gt[:,idx+2])


    difference_angles = []
    for qt, (gt_x, gt_y, gt_z, gt_w) in zip(track_quats, zip(gt_interpolated_x, gt_interpolated_y, gt_interpolated_z, gt_interpolated_w)):
        q_gt = Quaternion(gt_w, gt_x, gt_y, gt_z)
        diff = qt * q_gt.inverse
        difference_angles.append(abs(diff.degrees))


    rms = np.sqrt(np.mean(np.array(difference_angles)**2))

    print("median rotation error:", np.median(difference_angles), "°,  RMS:", rms, "°")

    return difference_angles

###############################################################################

def plot_error(track, gt, category, norm=lambda v: np.linalg.norm(v, axis=1), diff=None):
    idx = CSV_INDEXES_CAT[category]

    gt_poses_at_track = interpolate_gt(track[:, 1], gt[:, 0], gt[:, idx-1:idx+2])

    if diff is None:
        # use normal difference
        err_vec = track[:,idx:idx+3] - gt_poses_at_track
    else:
        err_vec = np.apply_along_axis(diff, axis=1, arr=np.column_stack((track[:,idx:idx+3], gt_poses_at_track)))

    if norm is not None:
        # calculate euclidean distances from difference vectors
        err = norm(err_vec)
    else:
        err = err_vec

    med = np.median(err)

    rms = np.sqrt(np.mean(err**2))
    print("median", category, "error:", med, "RMS:", rms)

    plt.plot(track[:,1], err)

    plt.xlabel('time [sec]')
    plt.title(category+' Error')

    return med


###############################################################################

def calc_error(track, gt, category, offset):
    idx = CSV_INDEXES_CAT[category]

    gt_poses_at_track = interpolate_gt(track[:, 1], gt[:, 0]+offset, gt[:, idx-1:idx+2])

    # use normal difference
    err_vec = track[:,idx:idx+3] - gt_poses_at_track

    err = np.linalg.norm(err_vec, axis=1)

    return np.sqrt(np.mean(err**2))

###############################################################################

def calc_rot_error(track, gt, offset):
    idx = CSV_INDEXES_CAT['Rotation']

    track_quats = to_quaternion_arr(track[:, idx:idx+4])

    # TODO: use proper quaternion interpolation instead of linear one
    gt_interpolated_x = np.interp(track[:,1], gt[:,0]+offset, gt[:,idx-1])
    gt_interpolated_y = np.interp(track[:,1], gt[:,0]+offset, gt[:,idx  ])
    gt_interpolated_z = np.interp(track[:,1], gt[:,0]+offset, gt[:,idx+1])
    gt_interpolated_w = np.interp(track[:,1], gt[:,0]+offset, gt[:,idx+2])


    difference_angles = []
    for qt, (gt_x, gt_y, gt_z, gt_w) in zip(track_quats, zip(gt_interpolated_x, gt_interpolated_y, gt_interpolated_z, gt_interpolated_w)):
        q_gt = Quaternion(gt_w, gt_x, gt_y, gt_z)
        diff = qt * q_gt.inverse
        difference_angles.append(abs(diff.degrees))

    return np.sqrt(np.mean(np.array(difference_angles)**2))

###############################################################################
# MAIN
###############################################################################

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process recorded poses')
    parser.add_argument('--folder', help='output folder -> paths for track and gt etc. will be determined automatically')
    parser.add_argument('--track', help='csv file containing the tracked poses')
    parser.add_argument('--gt',    help='csv file containing ground truth poses')
    parser.add_argument('--noshow', action='store_true', help='dont show graphs after plotting')
    parser.add_argument('--timeoffset', help='timing offset. Use "auto" to figure out offset in ground truth data by minimizing position error')
    parser.add_argument('--offsetrange', default=0.2, type=float, help='Range to sweep for auto timing offset calculation. Use small values for simulated (i.e. more accurate) data.')
    args = parser.parse_args()

    if not args.track:
        args.track = os.path.join(args.folder, 'track.csv')

    if not args.gt:
        args.gt = os.path.join(args.folder, 'ground_truth.csv')

    # create output folder for plots
    os.makedirs(os.path.join(args.folder, 'plots'), exist_ok=True)

    def savefigs(name):
        plt.tight_layout()
        plt.savefig(os.path.join(args.folder, 'plots', name+'.pgf'))
        plt.savefig(os.path.join(args.folder, 'plots', name+'.pdf'))



    # LOAD DATA
    ###########################################################################

    print("reading poses...")
    track = np.loadtxt(open(args.track, "rb"), delimiter=",", skiprows=1)

    # convert timestamps to relative times (so first sample is always at t=0)
    start_t = track[0,1]
    track[:,1] -= start_t

    print("got", track.shape[0], "poses")


    # ground truth data
    ########################################

    print("reading ground truth...")
    gt = np.loadtxt(open(args.gt, "rb"), delimiter=",", skiprows=1)
    if len(gt) == 0:
        gt = None
        print("no ground truth data :(")
    else:
        gt[:,0] -= start_t

        # remove ground truth where we don't have tracking
        gt = gt[0 <= gt[:,0]]
        gt = gt[gt[:,0] <= track[-1,1]]

        print("got", gt.shape[0], "ground truth poses")


    # TIMING OFFSET
    ###########################################################################

    if args.timeoffset == 'auto':
        print("trying to minimize error due to timing offset")

        offsets = np.linspace(-args.offsetrange, args.offsetrange, num=200)
        errs = []
        for offset in offsets:
            e = calc_error(track, gt, 'Position', offset)
            #e = calc_rot_error(track, gt, offset)
            #print("error at", offset, "=", e)
            errs.append(e)

        # fit polynomial
        p = np.polyfit(offsets, errs, 4)
        #print("polyfit:", p)

        pder  = np.polyder(p)
        pder2 = np.polyder(pder)

        # newton's method to find minima
        opt_offset = 0
        for i in range(20):
            opt_offset = opt_offset - np.polyval(pder, opt_offset)/np.polyval(pder2, opt_offset)

        print("optimal timing offset:", opt_offset)

        plt.plot(offsets, errs, label='error')
        plt.plot(offsets, np.polyval(p, offsets), label='polynomial fit')
        plt.axvline(x=opt_offset)
        plt.legend()
        plt.title('timing offset')
        plt.show()

        # correct data
        gt[:, 0] += opt_offset

    elif args.timeoffset:
        # correct data
        gt[:, 0] += float(args.timeoffset)


    # PLOT
    ###########################################################################

    figsize = (figure_width/25.4, figure_height/25.4) # convert mm to inches

    # Position
    ########################################

    plt.figure(figsize=figsize)
    plot_values(track, gt, 'Position')
    plt.ylabel('position [m]')
    plt.xlabel('time [sec]')
    plt.legend()

    savefigs('position_val')


    if gt:
        plt.figure(figsize=figsize)

        m = plot_error(track, gt, 'Position')
        plt.ylabel('position error [m]')
        plt.xlabel('time [sec]')
        plt.title('Position Error: median = {:.3f} mm'.format(m*1000))

        savefigs('position_err')


    # Orientation (Rotation)
    ########################################

    plt.figure(figsize=figsize)
    #plot_values(track, gt, 'Rotation')
    angles = plot_quaternions(track, gt)
    plt.ylabel('Angle [degrees]')
    plt.xlabel('time [sec]')
    plt.legend()

    savefigs('orientation_val')

    if gt:
        plt.figure(figsize=figsize)
        plt.plot(track[:,1], angles)
        plt.title('Orientation Error: median = {:.3f}°'.format(np.median(angles)))
        plt.ylabel('Angle [degrees]')
        plt.xlabel('time [sec]')

        savefigs('orientation_err')


    # Normalize Velocity
    # as it is only estimated up to scale!
    ########################################

    i = CSV_INDEXES_CAT['Lin. Velocity']

    # do we actually have ground truth data for the velocities?
    have_vel_gt = np.any(gt[:, i-1:])

    if have_vel_gt:

        # subsample ground truth
        gt_poses_at_track = interpolate_gt(track[:, 1], gt[:, 0], gt[:, i-1:i+5])

        # scale measured values to match ground-truth values (velocity is only estimated up to scale!)
        track[:, i:] = np.apply_along_axis(
                lambda r: r / np.linalg.norm(r),
                axis=1, arr=track[:, i:])

        gt_scale = np.reshape(np.linalg.norm(gt_poses_at_track, axis=1), (-1, 1))

        track[:, i:] = np.multiply(track[:, i:], np.repeat(gt_scale, 6, axis=1))

    else:
        print("WARNING: No ground-truth data available for velocities.")


    # Linear Velocity
    ########################################

    plt.figure(figsize=figsize)
    if have_vel_gt:
        plt.subplot(2,1,1)
        plot_values(track, gt, 'Lin. Velocity')
    else:
        plot_values(track, None, 'Lin. Velocity')

    plt.ylabel('velocity [m/s]')
    plt.legend()

    if have_vel_gt:
        plt.subplot(2,1,2)
        m = plot_error(track, gt, 'Lin. Velocity', diff=vector_angle, norm=None)
        plt.ylabel('angle [°]')
        plt.xlabel('time [sec]')
        plt.title('Velocity Direction Error: median = {:.3f}°'.format(m))

    savefigs('linear_velocity')


    # Angular Velocity
    ########################################

    # convert angular velocities to degree
    i = CSV_INDEXES_CAT['Ang. Velocity']
    track[:, i  :i+3] = np.rad2deg(track[:, i  :i+3])
    gt   [:, i-1:i+2] = np.rad2deg(gt   [:, i-1:i+2])

    plt.figure(figsize=figsize)
    if have_vel_gt:
        plt.subplot(2,1,1)
        plot_values(track, gt, 'Ang. Velocity')
    else:
        plot_values(track, None, 'Ang. Velocity')
    plt.ylabel('velocity [°/s]')
    plt.legend()

    if have_vel_gt:
        plt.subplot(2,1,2)
        m = plot_error(track, gt, 'Ang. Velocity', diff=vector_angle, norm=None)
        plt.ylabel('angle [°]')
        plt.xlabel('time [sec]')
        plt.title('Velocity Direction Error: median = {:.3f}°'.format(m))

    savefigs('angular_velocity')


    ########################################

    if not args.noshow:
        plt.show()
