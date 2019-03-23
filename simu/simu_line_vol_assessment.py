import cv2
import numpy as np
import sys,os
from matplotlib import pyplot as plt
from matplotlib import gridspec
from mpl_toolkits.mplot3d import Axes3D

# To read images
from scipy.misc import imread,imsave,imresize

def fit_z(z):
    return 525.0*0.12/z * 0.2
    # return np.exp(-(t-1)*0.4) * 20

# load text log
f = open('./simulate_line_vol.log', 'r')

i = 0
sP = np.zeros((100000, 3))
eP = np.zeros((100000, 3))
t = np.zeros((100000, 3))
q = np.zeros((100000, 4))
vol_st = np.zeros((100000, 49))
vol_ed = np.zeros((100000, 49))

for line in f:
    # print line
    data_tmp = line.split()
    # print data_tmp
    sP[i, 0:3]      = data_tmp[0:3]
    eP[i, 0:3]      = data_tmp[3:6]
    t[i, 0:3]       = data_tmp[6:9]
    q[i, 0:4]       = data_tmp[9:13]
    vol_st[i, 0:49]    = data_tmp[13:62]
    vol_ed[i, 0:49]    = data_tmp[62:111]

    i = i + 1


# i = 5000

sample_rate = 1

fig = plt.figure()
gs = gridspec.GridSpec(2, 2, width_ratios=[1, 1]) 

ax1 = plt.subplot(gs[0])
for pn in range(0, i, sample_rate):
    # ax1.scatter(np.ones((i, 1)) * pn, vol[:i, pn], color='r', s=2, marker='.', alpha=.4)
    # ax1.plot(range(0, 100, 2), vol[pn, :], color='r', marker='.', alpha=.4)
    if vol_st[pn, 0] > vol_st[pn, 1]:
    	ax1.plot(range(0, 98, 2), vol_st[pn, :], color='r', marker='.', alpha=.4)
    else:
    	ax1.plot(range(0, 98, 2), vol_st[pn, :], color='g', marker='.', alpha=.4)
    # ax1.set_yscale('log')
    # ax1.set_ylim(-plot_range, plot_range)
ax1.legend(shadow=True, fancybox=True)
ax1.set_xlabel('Portion of Line Being Cut (%)')
ax1.set_ylabel('Volume of Pose Covariance Matrix')
ax1.set_xlim(0, 100)

ax2 = plt.subplot(gs[1])
# labels = list('ABCD')
# data = []
# for cn in range(0, 20, 1):
#     data = [data, vol[0:i:sample_rate, cn]]
ax2.boxplot(vol_st[0:i:sample_rate, :])
ax2.set_yscale('log')
# ax2.set_xticklabels(['0', '5', '10', '15', '20', '25', '30', '35', '40', '45', '50', '55', '60', '65', '70', '75', '80', '85', '90', '95'])
# print data
# dataArr = np.asarray(data)
# print dataArr.shape
# ax2.boxplot(dataArr)
ax2.set_xlabel('Portion of Line Being Cut (%)')
ax2.set_ylabel('Volume of Pose Covariance Matrix')

ax3 = plt.subplot(gs[2])
for pn in range(0, i, sample_rate):
    if vol_ed[pn, 0] > vol_ed[pn, 1]:
    	ax3.plot(range(0, 98, 2), vol_ed[pn, :], color='r', marker='.', alpha=.4)
    else:
    	ax3.plot(range(0, 98, 2), vol_ed[pn, :], color='g', marker='.', alpha=.4)
    # ax1.set_yscale('log')
ax3.legend(shadow=True, fancybox=True)
ax3.set_xlabel('Portion of Line Being Cut (%)')
ax3.set_ylabel('Volume of Pose Covariance Matrix')
ax3.set_xlim(0, 100)

ax4 = plt.subplot(gs[3])
ax4.boxplot(vol_ed[0:i:sample_rate, :])
ax4.set_yscale('log')
# ax2.set_xticklabels(['0', '5', '10', '15', '20', '25', '30', '35', '40', '45', '50', '55', '60', '65', '70', '75', '80', '85', '90', '95'])
ax2.set_xlabel('Portion of Line Being Cut (%)')
ax2.set_ylabel('Volume of Pose Covariance Matrix')

plt.show()
