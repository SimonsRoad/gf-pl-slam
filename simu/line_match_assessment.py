import cv2
import numpy as np
import sys,os
from matplotlib import pyplot as plt
from matplotlib import gridspec
import matplotlib.lines as lines

# To read images
from scipy.misc import imread,imsave,imresize

# load text log - prev
# f = open('/home/yzhao/Data/22fa0d84-433c-482a-9aff-5b9caa4cb839/debug/line_no_projection/frame_line_info.log', 'r')
f = open('/home/yzhao/Data/96edfbf2-d999-490c-b1c8-d10291c6cce4/debug/matching/lbd/frame_line_info.log', 'r')

i = 0
frame_idx_1 = np.zeros((1000000, 1))
match_line_frame_1 = np.zeros((1000000, 1))
left_line_detect_1 = np.zeros((1000000, 1))
right_line_detect_1 = np.zeros((1000000, 1))

for line in f:
    # print line
    data_tmp = line.split()
    # print data_tmp
    frame_idx_1[i]        = data_tmp[0]
    match_line_frame_1[i]  = data_tmp[3]
    left_line_detect_1[i]  = data_tmp[4]
    right_line_detect_1[i]  = data_tmp[5]
    i = i + 1

# load text log - curr
# f = open('/home/yzhao/Data/22fa0d84-433c-482a-9aff-5b9caa4cb839/debug/line_full_pipeline/frame_line_info.log', 'r')
f = open('/home/yzhao/Data/96edfbf2-d999-490c-b1c8-d10291c6cce4/debug/matching/proj/frame_line_info.log', 'r')

i = 0
frame_idx_2 = np.zeros((1000000, 1))
match_line_frame_2 = np.zeros((1000000, 1))
left_line_detect_2 = np.zeros((1000000, 1))
right_line_detect_2 = np.zeros((1000000, 1))

for line in f:
    # print line
    data_tmp = line.split()
    # print data_tmp
    frame_idx_2[i]        = data_tmp[0]
    match_line_frame_2[i]  = data_tmp[3]
    left_line_detect_2[i]  = data_tmp[4]
    right_line_detect_2[i]  = data_tmp[5]
    i = i + 1


plot_range = 50

fig = plt.figure()
gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1]) 

ax1 = fig.add_subplot(gs[0])
# plot trend of line number
# plt.plot(frame_idx, num_line_match)
ax1.plot(frame_idx_1[:i], match_line_frame_1[:i], color='r', marker='.', alpha=.4, label="Bidirectional Best LBD")
# ax1.plot(frame_idx_1[:i], right_line_detect_1[:i], color='r', marker='.', alpha=.4)
ax1.plot(frame_idx_2[:i], match_line_frame_2[:i], color='g', marker='.', alpha=.4, label="Prediction + LBD + Distance Check")
# ax1.plot(frame_idx_2[:i], right_line_detect_2[:i], color='g', marker='.', alpha=.4)
# ax1.plot(frame_idx[:i], prev_line_stereo[:i], color='r', marker='.', alpha=.4, label="prev. num. stereo lines")
# ax1.plot(frame_idx[:i], curr_line_stereo[:i], color='g', marker='.', alpha=.4, label="curr. num. stereo lines")
# ax1.plot(frame_idx[:i], match_line_stereo[:i], color='b', marker='.', alpha=.4, label="num. matched stereo lines")
# ax1.set_yscale('log')
ax1.legend(shadow=True, fancybox=True)
ax1.set_xlabel('frame index')
ax1.set_ylabel('number of lines being matched cross-frame')
# ax1.set_xlim(-plot_range, plot_range)
# ax1.set_ylim(-plot_range, plot_range)

ax2 = fig.add_subplot(gs[1])
data=[match_line_frame_1[:i], match_line_frame_2[:i]]
ax2.boxplot(data)
ax2.set_ylabel('number of lines being matched cross-frame')

plt.show()



