import cv2
import numpy as np
import sys,os
from matplotlib import pyplot as plt

# To read images
from scipy.misc import imread,imsave,imresize

# import pcaflow.parameters as defaults
# from pcaflow.features.FeatureMatcherLibviso import FeatureMatcherLibviso
# from pcaflow.features.FeatureMatcherORB import FeatureMatcherORB
# from pcaflow.utils.show_correspondences import show_correspondences

def drawlines(img1,img2,lines,pts1,pts2):
    ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
    r,c,l = img1.shape
    # img1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
    # img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv2.line(img1, (x0,y0), (x1,y1), color,1)
        img1 = cv2.circle(img1,tuple(pt1),5,color,-1)
        img2 = cv2.circle(img2,tuple(pt2),5,color,-1)
    return img1,img2

def inverseMap(mapx, mapy):
    assert mapx.shape == mapy.shape
    mapx_inv = np.zeros(mapx.shape)
    mapy_inv = np.zeros(mapy.shape)
    for row in range(mapx.shape[0]):
        for col in range(mapx.shape[1]):
            dx = mapx[row][col]
            dy = mapy[row][col]
            print dx, dy
            mapx_inv[dx][dy] = row
            mapy_inv[dx][dy] = col
    return mapx_inv, mapy_inv

# load text log
f = open('/home/pang/workspace/leapcore/malleus/build/line_number.log', 'r')

i = 0
frame_idx = np.zeros((10000, 1))
num_line_prev = np.zeros((10000, 1))
num_line_curr = np.zeros((10000, 1))
num_line_match = np.zeros((10000, 1))

for line in f:
    # print line
    data_tmp = line.split()
    frame_idx[i] = data_tmp[0]
    num_line_prev[i] = data_tmp[1]
    num_line_curr[i] = data_tmp[2]
    num_line_match[i] = data_tmp[3]
    i = i + 1


# plot trend of line number
# plt.plot(frame_idx, num_line_match)
plt.plot(frame_idx[:i], num_line_curr[:i], 'bs--', label="number of stereo line")
plt.plot(frame_idx[:i], num_line_match[:i], 'rs--', label="number of matched line")
plt.legend(shadow=True, fancybox=True)
plt.show()