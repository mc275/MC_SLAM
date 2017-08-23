import numpy as np
import argparse
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

filepath = './';
# filename = filepath+'CameraFrameNavStateTrajectory2.txt';
# filename1 = filepath+'GroundTruth.txt';
# filename2 = filepath+'mono_IMU_align.txt';
# filename3 = filepath+'mono_align.txt';

parser = argparse.ArgumentParser(description='''
    This script plot EuRoC ground truth Trajectory and Mono+IMU SLAM Trajectory
    ''')

parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
parser.add_argument('second_file',
                    help='aligen mono IMU SLAM Trajectory trajectory (format: timestamp tx ty tz qx qy qz qw)')
parser.add_argument('third_file', help='Result image name')
args = parser.parse_args()

# NS = np.loadtxt(filename1, delimiter = ",");
NS1 = np.loadtxt(args.first_file);
NS2 = np.loadtxt(args.second_file);
# NS3 = np.loadtxt(filename3);


time = NS1[:, 0];
fig = plt.figure(1);
ax = fig.add_subplot(111, projection='3d');
ax.plot(NS2[:, 1], NS2[:, 2], NS2[:, 3], 'b', label=u'Mono+IMU-SLAM Trajectory');
# ax.plot(NS3[:,1],NS3[:,2],NS3[:,3],'y',label=u'Mono-SLAM Trajectory');
ax.plot(NS1[:, 1], NS1[:, 2], NS1[:, 3], 'r', label=u'Ground Truth');
plt.legend()

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Position in initial frame');
plt.savefig(filepath + args.third_file + '_Pw.eps', format="eps");

plt.show();
