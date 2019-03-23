# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

SeqIdxList =  [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 99];
Result_root = '/mnt/DATA/tmp/KITTI/PL_SLAM/Cut_PointLine_0.1/'
Num_Repeating = 5 # 1 # 10 # 20 # 

#----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ALERT = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

for iteration in range(0, Num_Repeating):

    Experiment_dir = Result_root + 'Round' + str(iteration + 1)
    cmd_mkdir = 'mkdir ' + Experiment_dir
    subprocess.call(cmd_mkdir, shell=True)

    for sn, SequenceIdx in enumerate(SeqIdxList):
        
        print bcolors.ALERT + "====================================================================" + bcolors.ENDC

        SeqIdx = str(SeqIdxList[sn]).zfill(2)
        print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqIdx

        if sn < 3:
            File_Setting = '../config/kitti/kitti00-02.yaml'
        elif sn == 3:
            File_Setting = '../config/kitti/kitti03.yaml'
        else:
            File_Setting = '../config/kitti/kitti04-10.yaml'
        
        File_stereo  = '/mnt/DATA/Datasets/Kitti_dataset/dataset/sequences/' + SeqIdx
        File_traj = Experiment_dir + '/' + SeqIdx

        cmd_slam   = str('../build/plslam_mod ' + 'kitti' + ' ' + File_Setting + ' ' + File_stereo + ' ' + File_traj)
        
        print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC

        print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
        # proc_slam = subprocess.Popen(cmd_slam, shell=True)
        subprocess.call(cmd_slam, shell=True)
