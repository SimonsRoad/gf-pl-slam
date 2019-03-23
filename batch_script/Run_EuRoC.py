# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# SeqNameList = ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult'];
# SeqNameList = ['V1_01_easy', 'V1_02_medium', 'V1_03_difficult', 'V2_01_easy', 'V2_02_medium', 'V2_03_difficult'];
SeqNameList = ['MH_01_easy'];
# Result_root = '/mnt/DATA/tmp/EuRoC/Cut_PointLine_1000_0.1/'
Result_root = '/home/pang/data/dataset/euroc/output/'
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

        File_Setting = '../config/euroc_params.yaml'
        File_stereo  = '/home/pang/data/dataset/euroc/' + SeqName + '/mav0'
        File_traj = Experiment_dir + '/' + SeqName # + '_blur_5'

        cmd_slam   = str('../build/plslam_mod ' + 'euroc' + ' ' + File_Setting + ' ' + File_stereo + ' ' + File_traj)
        
        print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC

        print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
        # proc_slam = subprocess.Popen(cmd_slam, shell=True)
        subprocess.call(cmd_slam, shell=True)
