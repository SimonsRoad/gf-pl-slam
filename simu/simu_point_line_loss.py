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
f = open('/home/yzhao/Eval/simulate_pl_loss_tX.log', 'r')

nPL = 0
nPP = 0
vPL = 0
vPP = 0
p3D = np.zeros((100000, 3))
lossPP = np.zeros((100000, 41))
lossPL = np.zeros((100000, 41))
gradPP = np.zeros((100000, 40))
gradPL = np.zeros((100000, 40))
varPP = np.zeros((100000, 40))
varPL = np.zeros((100000, 40))

for line in f:
    # print line
    data_tmp = line.split()
    # print data_tmp
    flg = int(data_tmp[0])
    # p3D[i/2, :] = data_tmp[1:4]

    if flg == 1:
        # check the flag as 1
        # feed pl loss
        lossPL[nPL, 0:41] = data_tmp[4:45]
        lossPL[nPL, 0:41] = lossPL[nPL, 0:41] / 10000.0
        gradPL[nPL, :] = lossPL[nPL, 1:41] - lossPL[nPL, 0:40]
        nPL = nPL + 1
    elif flg == 0:
        # check the flag as 0
        # feed pl loss
        lossPP[nPP, 0:41] = data_tmp[4:45]
        lossPP[nPP, 0:41] = lossPP[nPP, 0:41] / 10000.0
        gradPP[nPP, :] = lossPP[nPP, 1:41] - lossPP[nPP, 0:40]
        nPP = nPP + 1
    elif flg == 30:
        varPP[vPP, 0:20] = data_tmp[4:24]
        varPP[vPP, 20:40] = data_tmp[25:45]
        varPP[vPP, :] = 1.0 / (varPP[vPP, :] * 10000)
        vPP = vPP + 1
    else:
        varPL[vPL, 0:20] = data_tmp[4:24]
        varPL[vPL, 20:40] = data_tmp[25:45]
        varPL[vPL, :] = 1.0 / (varPL[vPL, :] * 10000 * 100)
        vPL = vPL + 1

# print varPP[:vPP, :]
# print varPL[:vPL, :]

i = 100

sample_rate = 1

fig = plt.figure()
gs = gridspec.GridSpec(3, 2, width_ratios=[1, 1]) 

ax1 = plt.subplot(gs[0])
for pn in range(0, i, sample_rate):
    # ax1.scatter(np.ones((i, 1)) * pn, vol[:i, pn], color='r', s=2, marker='.', alpha=.4)
    ax1.plot(range(-20, 21, 1), lossPL[pn, :], color='r', marker='.', alpha=.4)
    ax1.plot(range(-20, 21, 1), lossPP[pn, :], color='g', marker='.', alpha=.4)
    # ax1.set_yscale('log')
    # ax1.set_ylim(-plot_range, plot_range)
ax1.legend(shadow=True, fancybox=True)
ax1.set_xlabel('Pose Error (%)')
ax1.set_ylabel('Residual')
ax1.set_xlim(-20, 21)
ax1.legend(['point-to-line residual', 'point-to-point residual'])

ax2 = plt.subplot(gs[1])
# labels = list('ABCD')
# data = []
# for cn in range(0, 20, 1):
#     data = [data, vol[0:i:sample_rate, cn]]
bp1 = ax2.boxplot(lossPL[0:i:sample_rate, :])
plt.setp(bp1['boxes'], color='red')
bp2 = ax2.boxplot(lossPP[0:i:sample_rate, :])
# for patch in bp2['boxes']:
#     patch.set_facecolor('green')
plt.setp(bp2['boxes'], color='green')
# ax2.set_xticklabels(['0', '5', '10', '15', '20', '25', '30', '35', '40', '45', '50', '55', '60', '65', '70', '75', '80', '85', '90', '95'])
# print data
# dataArr = np.asarray(data)
# print dataArr.shape
# ax2.boxplot(dataArr)
ax2.set_xlabel('Pose Error (%)')
ax2.set_ylabel('Residual')
ax2.legend(['point-to-line residual', 'point-to-point residual'])

# gradient
ax3 = plt.subplot(gs[2])
for pn in range(0, i, sample_rate):
    ax3.plot(range(-20, 20, 1), gradPL[pn, :], color='r', marker='.', alpha=.4)
    ax3.plot(range(-20, 20, 1), gradPP[pn, :], color='g', marker='.', alpha=.4)
ax3.legend(shadow=True, fancybox=True)
ax3.set_xlabel('Pose Error (%)')
ax3.set_ylabel('Gradient of Residual')
ax3.set_xlim(-20, 20)
ax3.legend(['gradient of point-to-line residual', 'gradient of point-to-point residual'])

ax4 = plt.subplot(gs[3])
bp1 = ax4.boxplot(gradPL[0:i:sample_rate, :])
plt.setp(bp1['boxes'], color='red')
bp2 = ax4.boxplot(gradPP[0:i:sample_rate, :])
plt.setp(bp2['boxes'], color='green')
ax4.set_xlabel('Pose Error (%)')
ax4.set_ylabel('Gradient of Residual')
ax4.legend(['gradient of point-to-line residual', 'gradient of point-to-point residual'])


ax5 = plt.subplot(gs[4])
for pn in range(0, i, sample_rate):
    # ax1.scatter(np.ones((i, 1)) * pn, vol[:i, pn], color='r', s=2, marker='.', alpha=.4)
    ax5.plot(range(-20, 20, 1), varPL[pn, :], color='r', marker='.', alpha=.4)
    ax5.plot(range(-20, 20, 1), varPP[pn, :], color='g', marker='.', alpha=.4)
    # ax1.set_yscale('log')
    # ax1.set_ylim(-plot_range, plot_range)
ax5.legend(shadow=True, fancybox=True)
ax5.set_xlabel('Pose Error (%)')
ax5.set_ylabel('Variance of Residual')
ax5.set_xlim(-20, 21)
ax5.set_ylim(0, 1)
ax5.legend(['point-to-line residual', 'point-to-point residual'])

ax6 = plt.subplot(gs[5])
bp1 = ax6.boxplot(varPL[0:i:sample_rate, :], showfliers=False)
plt.setp(bp1['boxes'], color='red')
bp2 = ax6.boxplot(varPP[0:i:sample_rate, :], showfliers=False)
plt.setp(bp2['boxes'], color='green')
ax6.set_ylim(0, 1)
ax6.set_xlabel('Pose Error (%)')
ax6.set_ylabel('Variance of Residual')
ax6.legend(['point-to-line residual', 'point-to-point residual'])


plt.show()
