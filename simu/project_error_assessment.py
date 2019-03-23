import cv2
import numpy as np
import sys,os
from matplotlib import pyplot as plt
import matplotlib.lines as lines
from mpl_toolkits.mplot3d import Axes3D

# To read images
from scipy.misc import imread,imsave,imresize


# load text log
f = open('/home/yzhao/Data/75c27f4e-56f0-4d5d-b06b-405e85dede71/debug/evaluation_GT/proj_eval_res.log', 'r')

i = 0
frame_idx = np.zeros((1000000, 1))
line_prev_idx = np.zeros((1000000, 1))
line_curr_idx = np.zeros((1000000, 1))
sPt_u = np.zeros((1000000, 1))
sPt_v = np.zeros((1000000, 1))
ePt_u = np.zeros((1000000, 1))
ePt_v = np.zeros((1000000, 1))
sdist_Dline2Ppt = np.zeros((1000000, 1))
edist_Dline2Ppt = np.zeros((1000000, 1))
sdist_Dline2Tpt = np.zeros((1000000, 1))
edist_Dline2Tpt = np.zeros((1000000, 1))
sdist_Dpt2Pline = np.zeros((1000000, 1))
edist_Dpt2Pline = np.zeros((1000000, 1))
sdist_Dpt2Tline = np.zeros((1000000, 1))
edist_Dpt2Tline = np.zeros((1000000, 1))

for line in f:
    # print line
    data_tmp = line.split()
    # print data_tmp

    # print data_tmp[0]
    # if data_tmp[0] > 200:
    #     break

    frame_idx[i]        = data_tmp[0]
    line_prev_idx[i]    = data_tmp[1]
    line_curr_idx[i]    = data_tmp[2]
    sPt_u[i]    = data_tmp[3]
    sPt_v[i]    = data_tmp[4]
    ePt_u[i]    = data_tmp[5]
    ePt_v[i]    = data_tmp[6]
    sdist_Dline2Ppt[i]  = data_tmp[7]
    edist_Dline2Ppt[i]  = data_tmp[8]
    sdist_Dline2Tpt[i]  = data_tmp[9]
    edist_Dline2Tpt[i]  = data_tmp[10]
    sdist_Dpt2Pline[i]  = data_tmp[11]
    edist_Dpt2Pline[i]  = data_tmp[12]
    sdist_Dpt2Tline[i]  = data_tmp[13]    
    edist_Dpt2Tline[i]  = data_tmp[14]

    i = i + 1

print i

plot_range = 50 # 200 # 

sample_interval = 20

fig = plt.figure()

# ax1 = fig.add_subplot(121)
# ax1.scatter(frame_idx[:i], sdist_Dline2Tpt[:i], color='r', s=2, marker='.', alpha=.4, label="proj. err. of sPt")
# ax1.scatter(frame_idx[:i], edist_Dline2Tpt[:i], color='g', s=2, marker='.', alpha=.4, label="proj. err. of ePt")
# ax1.set_ylim(-plot_range, plot_range)

# ax2 = fig.add_subplot(122)
# ax2.scatter(frame_idx[:i], sdist_Dpt2Tline[:i], color='r', s=2, marker='.', alpha=.4, label="proj. err. of sPt")
# ax2.scatter(frame_idx[:i], edist_Dpt2Tline[:i], color='g', s=2, marker='.', alpha=.4, label="proj. err. of ePt")
# ax2.set_ylim(-plot_range, plot_range)

ax1 = fig.add_subplot(121)
ax1.plot([-plot_range, plot_range], [-plot_range, plot_range], 'k--', picker=2)
# plot trend of line number
# plt.plot(frame_idx, num_line_match)
ax1.scatter(sdist_Dline2Tpt[:i], sdist_Dline2Ppt[:i], color='r', s=10, marker='.', alpha=.4, label="proj. err. of sPt")
ax1.scatter(edist_Dline2Tpt[:i], edist_Dline2Ppt[:i], color='g', s=10, marker='.', alpha=.4, label="proj. err. of ePt")
ax1.legend(shadow=True, fancybox=True)
ax1.set_xlabel('ground truth point to detected line (pixel)')
ax1.set_ylabel('projected point to detected line (pixel)')
ax1.set_xlim(-plot_range, plot_range)
ax1.set_ylim(-plot_range, plot_range)

ax2 = fig.add_subplot(122)
ax2.plot([-plot_range, plot_range], [-plot_range, plot_range], 'k--', picker=2)
ax2.scatter(sdist_Dpt2Tline[:i], sdist_Dpt2Pline[:i], color='r', s=10, marker='.', alpha=.4, label="proj. err. of sPt")
ax2.scatter(edist_Dpt2Tline[:i], edist_Dpt2Pline[:i], color='g', s=10, marker='.', alpha=.4, label="proj. err. of ePt")
ax2.legend(shadow=True, fancybox=True)
ax2.set_xlabel('detected point to ground truth line (pixel)')
ax2.set_ylabel('detected point to projected line (pixel)')
ax2.set_xlim(-plot_range, plot_range)
ax2.set_ylim(-plot_range, plot_range)

# ax3 = fig.add_subplot(111, projection='3d')
# ax3.scatter(sPt_u[0:i:sample_interval], sPt_v[0:i:sample_interval], sdist_Dline2Ppt[0:i:sample_interval], color='r', marker='.', alpha=.4, label="proj. err. of sPt")
# ax3.scatter(ePt_u[0:i:sample_interval], ePt_v[0:i:sample_interval], edist_Dline2Ppt[0:i:sample_interval], color='g', marker='.', alpha=.4, label="proj. err. of ePt")
# ax3.legend(shadow=True, fancybox=True)
# ax3.set_xlabel('u (pixel)')
# ax3.set_ylabel('v (pixel)')
# ax3.set_zlabel('projected point to detected line (pixel)')

plt.show()