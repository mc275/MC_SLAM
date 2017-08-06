#!/usr/bin/python

import numpy as np
import argparse
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sys
import numpy
import argparse
import associate
import evaluate_ate

filepath = '/home/mc/My_Code/ORB_MC_VI/evaluate/';

parser = argparse.ArgumentParser(description='''
    This script plot EuRoC ground truth Trajectory and Mono+IMU SLAM Trajectory
    ''')

parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
parser.add_argument('second_file', help='IMU SLAM Trajectory trajectory (format: timestamp tx ty tz qx qy qz qw)')
#parser.add_argument('third_file', help='aligen mono IMU SLAM Trajectory trajectory (format: timestamp tx ty tz qx qy qz qw)')
parser.add_argument('image_name', help='Result image name')
parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)

args = parser.parse_args()


# load Data
#NS = np.loadtxt(filename1, delimiter = ",");
# NS1 = np.loadtxt(args.first_file);
# NS2 = np.loadtxt(args.second_file);


first_list = associate.read_file_list(args.first_file)
second_list = associate.read_file_list(args.second_file)

matches = associate.associate(first_list, second_list,float(args.offset),float(args.max_difference))    
if len(matches)<2:
    sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")


first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
second_xyz = numpy.matrix([[float(value) for value in second_list[b][0:3]] for a,b in matches]).transpose()
rot,trans,trans_error = evaluate_ate.align(second_xyz,first_xyz)

second_xyz_aligned = rot * second_xyz + trans
    
first_stamps = first_list.keys()
first_stamps.sort()
first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()
    
second_stamps = second_list.keys()
second_stamps.sort()
second_xyz_full = numpy.matrix([[float(value) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
second_xyz_full_aligned = rot * second_xyz_full + trans

print "compared_pose_pairs %d pairs"%(len(trans_error))

print "absolute_translational_error.rmse %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
print "absolute_translational_error.mean %f m"%numpy.mean(trans_error)
print "absolute_translational_error.median %f m"%numpy.median(trans_error)
print "absolute_translational_error.std %f m"%numpy.std(trans_error)
print "absolute_translational_error.min %f m"%numpy.min(trans_error)
print "absolute_translational_error.max %f m"%numpy.max(trans_error)


NS1 = np.array(first_xyz_full.transpose());
NS2 = np.array(second_xyz_full_aligned.transpose());


fig = plt.figure(1);
ax = fig.add_subplot(111, projection='3d');
ax.plot(NS1[:,0],NS1[:,1],NS1[:,2],'r',label=u'Ground Truth');
ax.plot(NS2[:,0],NS2[:,1],NS2[:,2],'b',label=u'Mono+IMU-SLAM Trajectory');
#ax.plot(NS3[:,1],NS3[:,2],NS3[:,3],'y',label=u'Mono-SLAM Trajectory');
plt.legend()


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Position in initial frame');
plt.savefig(filepath+args.image_name+'_Pw.eps', format="eps");

plt.show();