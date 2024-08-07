import os
import csv
from turtle import ycor
from unicodedata import name
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy

import numpy.linalg as LA
# import seaborn as sns
import cv2
import yaml


###### get config folder path from global config file #####
cwd = os.getcwd()
global_cfg_path = os.path.join(cwd, "configs", "global_config.yaml")
with open(global_cfg_path, 'r') as f:
    global_cfg = yaml.load(f, Loader=yaml.FullLoader)
LOCATION = global_cfg["location"]
TRACK_SPAN = global_cfg["track_width"]/2
config_folder = os.path.join(cwd, "configs")
###### get config folder path from global config file #####

WP_FILE_NAME = "wp.csv"

def PJcurvature(x, y):
    """
    input  : the coordinate of the three point
    output : the curvature and norm direction
    refer to https://github.com/Pjer-zhang/PJCurvature for detail
    """
    t_a = LA.norm([x[1] - x[0], y[1] - y[0]])
    t_b = LA.norm([x[2] - x[1], y[2] - y[1]])

    M = np.array([
        [1, -t_a, t_a ** 2],
        [1, 0, 0],
        [1, t_b, t_b ** 2]
    ])
    
    if LA.det(M) == 0:
        invM = LA.pinv(M)
    else:
        invM = LA.inv(M)
    a = np.matmul(invM, x)
    b = np.matmul(invM, y)

    kappa = 2 * (a[2] * b[1] - b[2] * a[1]) / (a[1] ** 2. + b[1] ** 2.) ** (1.5)
    return kappa, [b[1], -a[1]] / np.sqrt(a[1] ** 2. + b[1] ** 2.)


def show_wp(config_folder, location):
    wp_path = os.path.join(config_folder, location, WP_FILE_NAME)
    waypoints = np.loadtxt(wp_path, delimiter=',', skiprows=1)
    wp_x = waypoints[:, 0]
    wp_y = waypoints[:, 1]
    visualize_curvature_for_wp(wp_x, wp_y)
    # print(f"waypoints: {waypoints}, shape: {waypoints.shape}")
    # plt.axis('equal')
    # plt.plot(wp_x, wp_y, '-ro', markersize=0.1, label='waypoints')
    # plt.legend()
    # plt.show()

def read_wp(config_folder, location):
    wp_path = os.path.join(config_folder, location, WP_FILE_NAME)
    waypoints = np.loadtxt(wp_path, delimiter=',', skiprows=0)
    wp_x = waypoints[:, 0]
    wp_y = waypoints[:, 1]
    return wp_x, wp_y


def visualize_curvature_for_wp(wp_x, wp_y, overtaking_thres=0.05, corner_thres = 0.1, overtaking_gap=5, corner_gap=5):
    kappa = []
    no = []
    po = []
    ka = []
    for idx in range(len(wp_y))[1:-2]:
        x = wp_x[idx - 1:idx + 2]
        y = wp_y[idx - 1:idx + 2]
        kappa, norm = PJcurvature(x, y)
        ka.append(kappa)
        no.append(norm)
        po.append([x[1], y[1]])

    po = np.array([[wp_x[0], wp_y[1]]]+po+[[wp_x[-2], wp_y[-2]]]+[[wp_x[-1], wp_y[-1]]])
    no = np.array([no[0]]+no+[no[-2]]+[no[-1]])
    ka = np.array([ka[0]]+ka+[ka[-2]]+[ka[-1]])

    # overtaking
    i = 0
    n_ka = len(ka)
    segments = []
    while i < n_ka:
        if abs(ka[i]) < overtaking_thres:
            begin = i
            while i < n_ka and abs(ka[i]) < overtaking_thres:
                i += 1
            if i-begin > overtaking_gap:
                segments.append((begin, i-1))
        else:
            i += 1
    overtaking_idx = []
    for seg in segments:
        b, e = seg[0], seg[1]
        i = b
        while i <= e:
            overtaking_idx.append(i)
            i += 1
    print("overtaking idx: ")
    print(overtaking_idx)
    idx_path = os.path.join(config_folder, LOCATION, 'overtaking_wp_idx')
    np.save(idx_path, np.array(overtaking_idx))

    # corner
    i = 0
    n_ka = len(ka)
    segments = []
    while i < n_ka:
        if abs(ka[i]) > corner_thres:
            begin = i
            while i < n_ka and abs(ka[i]) > corner_thres:
                i += 1
            if i-begin > corner_gap:
                segments.append((begin, i-1))
        else:
            i += 1
    corner_idx = []
    for seg in segments:
        b, e = seg[0], seg[1]
        i = b
        while i <= e:
            corner_idx.append(i)
            i += 1
    print("corner idx: ")
    print(corner_idx)
    idx_path = os.path.join(config_folder, LOCATION, 'corner_wp_idx')
    np.save(idx_path, np.array(corner_idx))

    # speed up zone
    i = 0
    n_ka = len(ka)
    fast_zone_seg = [(-5, 20, 15, 55), (0, 15, 25, 25), (-40, 27, -25, 35),  (50, 20, 60, 60)]
    fast_zone_multi = [1.5, 1.5, 1.5, 1.2]
    slow_zone_seg = []
    slow_zone_multi = []
    fast_idx = []
    slow_idx = []

    # open wp.csv and multiply the speed of the waypoints in the fast zone
    wp_path = os.path.join(config_folder, LOCATION, WP_FILE_NAME)
    waypoints = np.loadtxt(wp_path, delimiter=',')

    while i < n_ka:
        for j in range(len(fast_zone_seg)):
            zone = fast_zone_seg[j]
            if wp_x[i] > zone[0] and wp_x[i] < zone[2] and wp_y[i] > zone[1] and wp_y[i] < zone[3] and i not in fast_idx:
                fast_idx.append(i)
                waypoints[i, 2] *= fast_zone_multi[j]
        i += 1
    print("fast idx: ")
    print(fast_idx)
    new_wp_path = os.path.join(config_folder, LOCATION, 'wp_speedzone.csv')
    with open(new_wp_path, 'w') as f:
        for i in range(len(waypoints)):
            f.write(f'{waypoints[i, 0]},{waypoints[i, 1]},{waypoints[i, 2]}\n')




    fig = plt.figure(figsize=(8, 5), dpi=120)
    ax = fig.add_subplot(2, 1, 1)
    label_straight = False
    label_corner = False
    label_fast = False
    straight_corner_pt_size = 100
    zone_size = 400
    for i in fast_idx:
        if not label_fast:
            plt.scatter(po[i, 0], po[i, 1], c='g', label='fast_wp', s=zone_size)
            label_fast = True
        else:
            plt.scatter(po[i, 0], po[i, 1], c='g', s=zone_size)
    for i in overtaking_idx:
        if not label_straight:
            plt.scatter(po[i, 0], po[i, 1], c='r', label='straight_wp', s=straight_corner_pt_size)
            label_straight = True
        else:
            plt.scatter(po[i, 0], po[i, 1], c='r', s=straight_corner_pt_size)
    for i in corner_idx:
        if not label_corner:
            plt.scatter(po[i, 0], po[i, 1], c='b', label='corner_wp', s=straight_corner_pt_size)
            label_corner = True
        else:
            plt.scatter(po[i, 0], po[i, 1], c='b', s=straight_corner_pt_size)

    plt.plot(po[:, 0], po[:, 1])
    plt.quiver(po[:, 0], po[:, 1], ka * no[:, 0], ka * no[:, 1], label='curvature')
    plt.legend()
    plt.axis('equal')


    ax = fig.add_subplot(2, 1, 2)
    plt.plot(ka, '-bo', markersize=0.1)
    plt.show()


def save_wp_with_width(cwd, config_folder, location):
    wp_x, wp_y = read_wp(config_folder, location)
    new_wp_file_path = os.path.join(cwd, 'scripts', 'trajectory_generator', 
                                    'outputs', f'{location}_wp_w_width.csv')
    print(new_wp_file_path)
    with open(new_wp_file_path, 'w') as f:
        f.write('#x_m, y_m, w_tr_right_m, w_tr_left_m \n')
        for i in range(len(wp_x)):
            f.write(f'{wp_x[i]},{wp_y[i]},{TRACK_SPAN},{TRACK_SPAN}\n')         


if __name__ == "__main__":
    cwd = os.getcwd()
    config_folder = os.path.join(cwd, 'configs')
    show_wp(config_folder, LOCATION)
    # visualize_curvature_for_wp(config_folder, LOCATION)
    save_wp_with_width(cwd, config_folder, LOCATION)

