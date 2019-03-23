import cv2
import numpy as np
import sys,os
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# To read images
from scipy.misc import imread,imsave,imresize

def fit_z(z):
    return 525.0*0.12/z * 0.2
    # return np.exp(-(t-1)*0.4) * 20

# load text log
# f = open('/home/yzhao/Data/75c27f4e-56f0-4d5d-b06b-405e85dede71/debug/evaluation_GT/stereo_eval_res.log', 'r')
f = open('/home/yzhao/Data/31f464f0-065f-4b3e-aeb0-25bcd4d9eea6/debug/evaluation_GT/stereo_eval_res.log', 'r')

i = 0
frame_idx = np.zeros((1000000, 1))
line_idx = np.zeros((1000000, 1))
sPt_u = np.zeros((1000000, 1))
sPt_v = np.zeros((1000000, 1))
sPt_disp = np.zeros((1000000, 1))
sPt_disp_err = np.zeros((1000000, 1))
ePt_u = np.zeros((1000000, 1))
ePt_v = np.zeros((1000000, 1))
ePt_disp = np.zeros((1000000, 1))
ePt_disp_err = np.zeros((1000000, 1))
sPt_x = np.zeros((1000000, 1))
sPt_y = np.zeros((1000000, 1))
sPt_z = np.zeros((1000000, 1))
sPt_x_err = np.zeros((1000000, 1))
sPt_y_err = np.zeros((1000000, 1))
sPt_z_err = np.zeros((1000000, 1))
ePt_x = np.zeros((1000000, 1))
ePt_y = np.zeros((1000000, 1))
ePt_z = np.zeros((1000000, 1))
ePt_x_err = np.zeros((1000000, 1))
ePt_y_err = np.zeros((1000000, 1))
ePt_z_err = np.zeros((1000000, 1))

for line in f:
    # print line
    data_tmp = line.split()
    # print data_tmp
    frame_idx[i]    = data_tmp[0]
    line_idx[i]     = data_tmp[1]
    sPt_u[i]        = data_tmp[2]
    sPt_v[i]        = data_tmp[3]
    sPt_disp[i]     = data_tmp[4]
    sPt_disp_err[i] = data_tmp[5]
    ePt_u[i]        = data_tmp[6]
    ePt_v[i]        = data_tmp[7]
    ePt_disp[i]     = data_tmp[8]
    ePt_disp_err[i] = data_tmp[9]    
    sPt_x[i]        = data_tmp[10]
    sPt_y[i]        = data_tmp[11]
    sPt_z[i]        = data_tmp[12]
    sPt_x_err[i]    = data_tmp[13]
    sPt_y_err[i]    = data_tmp[14]
    sPt_z_err[i]    = data_tmp[15]
    ePt_x[i]        = data_tmp[16]
    ePt_y[i]        = data_tmp[17]
    ePt_z[i]        = data_tmp[18]
    ePt_x_err[i]    = data_tmp[19]
    ePt_y_err[i]    = data_tmp[20]
    ePt_z_err[i]    = data_tmp[21]

    i = i + 1


fig = plt.figure()

plot_range = 30

# ax1 = fig.add_subplot(221)
# ax1.scatter(frame_idx[:i], sPt_disp_err[:i], color='r', s=2, marker='.', alpha=.4, label="disp. err. of sPt")
# ax1.scatter(frame_idx[:i], ePt_disp_err[:i], color='g', s=2, marker='.', alpha=.4, label="disp. err. of ePt")

# ax2 = fig.add_subplot(222)
# ax2.scatter(frame_idx[:i], sPt_x_err[:i], color='r', s=2, marker='.', alpha=.4, label="x err. of sPt")
# ax2.scatter(frame_idx[:i], ePt_x_err[:i], color='g', s=2, marker='.', alpha=.4, label="x err. of ePt")

# ax3 = fig.add_subplot(223)
# ax3.scatter(frame_idx[:i], sPt_y_err[:i], color='r', s=2, marker='.', alpha=.4, label="y err. of sPt")
# ax3.scatter(frame_idx[:i], ePt_y_err[:i], color='g', s=2, marker='.', alpha=.4, label="y err. of ePt")

# ax4 = fig.add_subplot(224)
# ax4.scatter(frame_idx[:i], sPt_z_err[:i], color='r', s=2, marker='.', alpha=.4, label="z err. of sPt")
# ax4.scatter(frame_idx[:i], ePt_z_err[:i], color='g', s=2, marker='.', alpha=.4, label="z err. of ePt")


# plot trend of line number
# plt.plot(frame_idx, num_line_match)



# ax1 = fig.add_subplot(231)
# ax1.scatter(sPt_u[:i], sPt_disp_err[:i], color='r', s=2, marker='.', alpha=.4, label="disp. err. of sPt")
# ax1.scatter(ePt_u[:i], ePt_disp_err[:i], color='g', s=2, marker='.', alpha=.4, label="disp. err. of ePt")
# ax1.legend(shadow=True, fancybox=True)
# ax1.set_xlim(0, 640)
# ax1.set_ylim(-plot_range, plot_range)
# ax1.set_xlabel('u (pixel)')
# ax1.set_ylabel('disparity error (pixel)')

# ax2 = fig.add_subplot(232)
# ax2.scatter(sPt_v[:i], sPt_disp_err[:i], color='r', s=2, marker='.', alpha=.4, label="disp. err. of sPt")
# ax2.scatter(ePt_v[:i], ePt_disp_err[:i], color='g', s=2, marker='.', alpha=.4, label="disp. err. of ePt")
# ax2.legend(shadow=True, fancybox=True)
# ax2.set_xlim(0, 480)
# ax2.set_ylim(-plot_range, plot_range)
# ax2.set_xlabel('v (pixel)')
# ax2.set_ylabel('disparity error (pixel)')

ax3 = fig.add_subplot(121)
ax3.scatter(sPt_disp[:i], sPt_disp_err[:i], color='r', s=2, marker='.', alpha=.4, label="disp. err. of sPt")
ax3.scatter(ePt_disp[:i], ePt_disp_err[:i], color='g', s=2, marker='.', alpha=.4, label="disp. err. of ePt")
ax3.legend(shadow=True, fancybox=True)
ax3.set_xlim(0, 60)
ax3.set_ylim(-plot_range, plot_range)
ax3.set_xlabel('disparity (pixel)')
ax3.set_ylabel('disparity error (pixel)')
d=np.arange(0.1, 60.0, 0.1)
de=fit_z(525.0*0.12/d)
ax3.plot(d, de, 'k')
ax3.plot(d, -de, 'k')

ax4 = fig.add_subplot(122)
ax4.scatter(sPt_z[:i], sPt_disp_err[:i], color='r', s=2, marker='.', alpha=.4, label="disp. err. of sPt")
ax4.scatter(ePt_z[:i], ePt_disp_err[:i], color='g', s=2, marker='.', alpha=.4, label="disp. err. of ePt")
ax4.legend(shadow=True, fancybox=True)
ax4.set_xlim(0, 15)
ax4.set_ylim(-plot_range, plot_range)
ax4.set_xlabel('z (m)')
ax4.set_ylabel('disparity error (pixel)')
z=np.arange(0.0, 15.0, 0.1)
de=fit_z(z)
ax4.plot(z, de, 'k')
ax4.plot(z, -de, 'k')

# ax5 = fig.add_subplot(235)
# ax5.scatter(sPt_z[:i], sPt_z_err[:i], color='r', s=2, marker='.', alpha=.4, label="z. err. of sPt")
# ax5.scatter(ePt_z[:i], ePt_z_err[:i], color='g', s=2, marker='.', alpha=.4, label="z. err. of ePt")
# ax5.legend(shadow=True, fancybox=True)
# ax5.set_xlim(0, 15)
# ax5.set_ylim(-plot_range / 3, plot_range / 3)
# ax5.set_xlabel('z (m)')
# ax5.set_ylabel('z error (m)')



# ax1 = fig.add_subplot(121)
# ax1.scatter(sPt_u[:i], sPt_z_err[:i], color='r', s=2, marker='^', alpha=.4, label="z. err. of sPt")
# ax1.scatter(ePt_u[:i], ePt_z_err[:i], color='g', s=2, marker='*', alpha=.4, label="z. err. of ePt")
# ax1.legend(shadow=True, fancybox=True)
# ax1.set_ylim(-plot_range, plot_range)

# ax2 = fig.add_subplot(122)
# ax2.scatter(sPt_v[:i], sPt_z_err[:i], color='r', s=2, marker='^', alpha=.4, label="z. err. of sPt")
# ax2.scatter(ePt_v[:i], ePt_z_err[:i], color='g', s=2, marker='*', alpha=.4, label="z. err. of ePt")
# ax2.legend(shadow=True, fancybox=True)
# ax2.set_ylim(-plot_range, plot_range)

# sample_interval = 5 # 20

# # # ax3 = fig.add_subplot(111, projection='3d')
# # # datamin=min(sPt_disp_err[:i])
# # # datamax=max(sPt_disp_err[:i])
# # # ax3.scatter(sPt_u[0:i:sample_interval], sPt_v[0:i:sample_interval], sPt_disp_err[0:i:sample_interval], c=sPt_disp_err[0:i:sample_interval], vmin=datamin, vmax=datamax, marker='.', alpha=.4, label="disp. err. of sPt")
# # # datamin=min(ePt_disp_err[:i])
# # # datamax=max(ePt_disp_err[:i])
# # # ax3.scatter(ePt_u[0:i:sample_interval], ePt_v[0:i:sample_interval], ePt_disp_err[0:i:sample_interval], c=ePt_disp_err[0:i:sample_interval], vmin=datamin, vmax=datamax, marker='.', alpha=.4, label="disp. err. of ePt")
# # # ax3.axis('equal')
# # # ax3.legend(shadow=True, fancybox=True)
# # # ax3.set_xlabel('u (pixel)')
# # # ax3.set_ylabel('v (pixel)')
# # # ax3.set_zlabel('disparity error (pixel)')


# ax4 = fig.add_subplot(111, projection='3d')
# ax4.scatter(sPt_x_err[0:i:sample_interval], sPt_y_err[0:i:sample_interval], sPt_z_err[0:i:sample_interval], c='r', marker='.', alpha=.4, label="SE3-x err. of sPt")
# ax4.scatter(ePt_x_err[0:i:sample_interval], ePt_y_err[0:i:sample_interval], ePt_z_err[0:i:sample_interval], c='g', marker='.', alpha=.4, label="SE3-x err. of ePt")
# ax4.axis('equal')
# ax4.legend(shadow=True, fancybox=True)
# ax4.set_xlabel('error on x (m)')
# ax4.set_ylabel('error on y (m)')
# ax4.set_zlabel('error on z (m)')

# plt.show()

# plt.scatter(sPt_x[:i], sPt_x_err[:i], color='r', s=2, marker='^', alpha=.4, label="SE3-x err. of sPt")
# plt.scatter(ePt_x[:i], ePt_x_err[:i], color='g', s=2, marker='*', alpha=.4, label="SE3-x err. of ePt")
# plt.legend(shadow=True, fancybox=True)
# plt.show()

# plt.scatter(sPt_y[:i], sPt_y_err[:i], color='r', s=2, marker='^', alpha=.4, label="SE3-y err. of sPt")
# plt.scatter(ePt_y[:i], ePt_y_err[:i], color='g', s=2, marker='*', alpha=.4, label="SE3-y err. of ePt")
# plt.legend(shadow=True, fancybox=True)
# plt.show()

# plt.scatter(sPt_z[:i], sPt_z_err[:i], color='r', s=2, marker='^', alpha=.4, label="SE3-z err. of sPt")
# plt.scatter(ePt_z[:i], ePt_z_err[:i], color='g', s=2, marker='*', alpha=.4, label="SE3-z err. of ePt")
# plt.legend(shadow=True, fancybox=True)
plt.show()