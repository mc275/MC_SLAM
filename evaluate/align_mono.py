#!/usr/bin/python
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import argparse
import math

# filepath = '/home/mc/SLAM/evaluate/';
# filename = filepath+'data.txt';
# filename1 = filepath+'CameraFrameNavStateTrajectory.txt';
# filename2 = filepath+'mono_CameraTrajectory.txt';
# filename3 = filepath+'ExtrixParameter.txt';


parser = argparse.ArgumentParser(description='''
    This script align EuRoC ground truth time with VI Trajectory
    ''')

parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
parser.add_argument('second_file', help='aligen time ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
args = parser.parse_args()


NS = np.loadtxt(args.first_file, delimiter = ",");

NS[:,0] = NS[:,0]/1e9;

np.savetxt(args.second_file, NS, delimiter=' ', fmt='%f') 


# mono_traj = np.loadtxt(filename2);
# ExtriParm = np.loadtxt(filename3);

# rot  = ExtriParm[:, 0:3].reshape(3,3);
# trans = ExtriParm[:, 3].reshape(3,1);
# xyz = mono_traj[:, 0:4];
# align_gt = xyz;
# for i in range(0, (xyz.shape)[0]-1):
# 	a = xyz[i, 1:4].reshape(3,1);
# 	align_gt[i,1:4] = np.transpose(np.dot(rot, a)+trans);

# np.savetxt('mono_align.txt', align_gt, delimiter=' ', fmt='%f') 


