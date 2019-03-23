import cv2
import numpy as np
import sys,os
from matplotlib import pyplot as plt
from matplotlib import gridspec
import matplotlib.lines as lines

# To read images
from scipy.misc import imread,imsave,imresize

# load text log - prev
f = open('/home/yzhao/Data/96edfbf2-d999-490c-b1c8-d10291c6cce4/debug/lbd/frame_line_info.log', 'r')

i = 0
frame_idx_1 = np.zeros((1000000, 1))
match_line_stereo_1 = np.zeros((1000000, 1))
left_line_detect_1 = np.zeros((1000000, 1))
right_line_detect_1 = np.zeros((1000000, 1))

for line in f:
    # print line
    data_tmp = line.split()
    # print data_tmp
    frame_idx_1[i]        = data_tmp[0]
    match_line_stereo_1[i]  = data_tmp[2]
    left_line_detect_1[i]  = data_tmp[4]
    right_line_detect_1[i]  = data_tmp[5]
    i = i + 1

# load text log - curr
f = open('/home/yzhao/Data/96edfbf2-d999-490c-b1c8-d10291c6cce4/debug/ssd/frame_line_info.log', 'r')

j = 0
frame_idx_2 = np.zeros((1000000, 1))
match_line_stereo_2 = np.zeros((1000000, 1))
left_line_detect_2 = np.zeros((1000000, 1))
right_line_detect_2 = np.zeros((1000000, 1))

for line in f:
    # print line
    data_tmp = line.split()
    # print data_tmp
    frame_idx_2[j]        = data_tmp[0]
    match_line_stereo_2[j]  = data_tmp[2]
    left_line_detect_2[j]  = data_tmp[4]
    right_line_detect_2[j]  = data_tmp[5]
    j = j + 1


plot_range = 50

fig = plt.figure()
gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1]) 

ax3 = plt.subplot(gs[0])
# plot trend of line number
ax3.plot(frame_idx_1[:i], match_line_stereo_1[:i], color='r', marker='.', alpha=.4, label="lines stereo matched with LBD")
ax3.plot(frame_idx_2[:j], match_line_stereo_2[:j], color='g', marker='.', alpha=.4, label="lines stereo matched with SSD")
ax3.legend(shadow=True, fancybox=True)
ax3.set_xlabel('frame index')
ax3.set_ylabel('number of lines')
ax3.set_xlim(0, frame_idx_2[j-1])
# ax1.set_ylim(-plot_range, plot_range)

ax4 = plt.subplot(gs[1])
data=[match_line_stereo_1[:i], match_line_stereo_2[:j]]
ax4.boxplot(data)
ax4.set_ylabel('number of lines')


plt.show()

