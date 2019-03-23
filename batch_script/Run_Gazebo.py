# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

SeqNameList = ['hard_wood_0.01'];
# SeqNameList = ['brick_wall_0.01', 'hard_wood_0.01', 'wood_wall_0.01']; # ['hard_wood_0.01']; # 
# Result_root = '/mnt/DATA/tmp/Gazebo/PointLine_maxVol_0.30/'
Result_root = '/mnt/DATA/tmp/LineCut/Gazebo/'
Num_Repeating = 1 # 10 # 20 # 5 # 

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
    cmd_mkdir = 'mkdir -p ' + Experiment_dir
    subprocess.call(cmd_mkdir, shell=True)

    for sn, sname in enumerate(SeqNameList):
        
        print bcolors.ALERT + "====================================================================" + bcolors.ENDC

        SeqName = SeqNameList[sn]
        print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

        File_Setting = '../config/gazebo_params.yaml'
        File_stereo  = '/mnt/DATA/Datasets/GazeboMaze/' + SeqName
        File_traj = Experiment_dir + '/' + SeqName

        cmd_slam   = str('../build/plslam_mod ' + 'gazebo' + ' ' + File_Setting + ' ' + File_stereo + ' ' + File_traj)
        
        print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC

        print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
        # proc_slam = subprocess.Popen(cmd_slam, shell=True)
        subprocess.call(cmd_slam, shell=True)
