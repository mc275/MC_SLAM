#-*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

filepath = '/home/mc/My_Code/ORB_MC_VI/tmp/';


biasa = np.loadtxt(filepath+'biasa.txt');
plt.figure(1);
p11, =plt.plot(biasa[:,0]-biasa[0,0],biasa[:,1]);
p12, =plt.plot(biasa[:,0]-biasa[0,0],biasa[:,2]);
p13, =plt.plot(biasa[:,0]-biasa[0,0],biasa[:,3]);
plt.title('Acc-Bias');
plt.xlabel('Time (s)')
plt.ylabel('accelerometer bias ($\mathrm{m}/\mathrm{s}^2$)')
plt.legend([p11,p12,p13],["x","y","z"],loc=4);
plt.savefig(filepath+"biasa.eps", format="eps")
#plt.legend(p12,'y');
#plt.legend(p13,'z');

scale = np.loadtxt(filepath+'scale.txt');
plt.figure(2);
# [p21,p22] = plt.plot(scale[:,0]-scale[0,0],scale[:,1:3]);
p21 = plt.plot(scale[:,0]-scale[0,0],scale[:,2]);
plt.title('Trajectory Scale');
plt.xlabel('Time (s)')
plt.ylabel('Scale Factor')
#plt.legend([p21],['aftopt','befopt']);
plt.savefig(filepath+'/scale.eps', format="eps")

condnum = np.loadtxt(filepath+'condnum.txt');
plt.figure(3);
plt.plot(condnum[:,0]-condnum[0,0],condnum[:,1]/condnum[:,6]);
plt.title('Condition number');
plt.savefig(filepath+'condnum.eps', format="eps")


biasg = np.loadtxt(filepath+'biasg.txt');
plt.figure(4);
p41, =plt.plot(biasg[:,0]-biasg[0,0],biasg[:,1]);
p42, =plt.plot(biasg[:,0]-biasg[0,0],biasg[:,2]);
p43, =plt.plot(biasg[:,0]-biasg[0,0],biasg[:,3]);
plt.title('Gyr-Bias');
plt.xlabel('Time (s)')
plt.ylabel('gyroscope bias ($\mathrm{rad}/\mathrm{s}$)')
plt.legend([p41,p42,p43],["x","y","z"],loc=4);
plt.savefig(filepath+"biasg.eps", format="eps")

computetime = np.loadtxt(filepath+'condnum.txt');
plt.figure(5);
plt.plot(computetime[:,0]-computetime[0,0],computetime[:,1]);
plt.title('Processing Time');
plt.xlabel('Time (s)')
plt.ylabel('Processing Time (ms)')
plt.ylim(ymin=3)
plt.savefig(filepath+'computetime.eps', format="eps")

plt.show();
