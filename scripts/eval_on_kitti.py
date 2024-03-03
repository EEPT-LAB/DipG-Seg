#!/usr/bin/python2.7
# -*- coding: UTF-8 -*-

import rospy
import os
import numpy as np
import pandas as pd
import time
import roslaunch
import signal
from datetime import datetime
import sys

rospy.init_node('eval_node', anonymous=True, disable_signals=True)
global seq_list, result_all, csv_name, result_files_to_folder

# seq 01 is not used, and the reason is explained in the paper.
seq_list = ["00","02","03","04","05","06","07","08","09","10"]

result_files_to_folder = []
result_all=[]
csv_name = "evaluation_res_"
def start_one_seq(seq):   
    #---------------------------------run segmentation-----------------------------------
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    node = roslaunch.core.Node(package="dipgseg", node_type="offline_kitti_node", name="dipgseg_node_single_final_"+seq, namespace='/',
                                machine_name=None, args=seq,
                                respawn=False, respawn_delay=0.0,
                                remap_args=None, env_args=None, output=None, cwd=None,
                                launch_prefix=None, required=False, filename='<unknown>')
    process = launch.launch(node)
    print("start a new seq------------------------------------------------>"+seq)
    return process
    #------------------------------run evaluate-----------------------------------------------

# This function will test all the sequences in seq_list and save the result in the "output_path" folder,
# which is defined in the launch file.
def one_batch_test():
    global seq_list, result_all, csv_name
    result_all = []
    result_temp = []
    for seq in seq_list:
        p = start_one_seq(seq)
        while p.is_alive():
            time.sleep(2)
        p.stop()

    result_path = rospy.get_param("output_path")
    for seq in seq_list:
        result_file_name = result_path+seq+".csv"
        try:
            result_seq = np.loadtxt(result_file_name, delimiter=",")
            result_files_to_folder.append(result_file_name)
            # os.remove(result_file_name)
            if len(result_temp) == 0:
                result_temp = result_seq
            else:
                result_temp = np.vstack((result_temp, result_seq))
        except: pass
    for i in range(result_temp.shape[0]):
        tp, tn, fn, fp = 0, 0, 0, 0
        if result_temp.shape[0] == 1:
            tp=result_temp[4]
            tn=result_temp[1]
            fn=result_temp[2]
            fp=result_temp[3]
        else:
            tp=result_temp[i,4]
            tn=result_temp[i,1]
            fn=result_temp[i,2]
            fp=result_temp[i,3]
        precision=float(tp)/float((tp+fp))
        acc=float(tp+tn)/(float)(tp+fp+tn+fn)
        recall=float(tp)/(tp+fn)
        f1_score=2*float(precision*recall)/(precision+recall)
        miou = 0.5*(float(tp)/float(tp+fn+fp)+float(tn)/float(tn+fn+fp))
        seq_num = result_temp[i,0]
        if len(result_all)==0:
            result_all = np.array([[seq_num, tn, fn, fp, tp, precision, recall, f1_score, acc, miou]])
        else:
            result_seq = np.array([[seq_num, tn, fn, fp, tp, precision, recall, f1_score, acc, miou]])
            result_all = np.vstack((result_all,result_seq))

    result_all = result_all[np.argsort(result_all[:,0])]
    mean = np.mean(result_all, axis=0)
    std = np.std(result_all, axis=0, ddof=1)
    result_all = np.vstack((result_all,mean))
    result_all = np.vstack((result_all,std))
    df = pd.DataFrame(result_all, columns=['Seq', 'TN', 'FN', 'FP', 'TP', "Precision", "Recall", "F1", "Acc", "mIoU"])
    df.iloc[-1][1:5]=-100
    df.iloc[-2][1:5]=-100
    df=df.replace(-100, np.nan)
    df["Seq"] = df["Seq"].astype(str)
    df.at[result_temp.shape[0], "Seq"]="Average"
    df.at[result_temp.shape[0]+1, "Seq"]="STD"
    now = datetime.now()
    now_fs = now.strftime( '%Y-%m-%d-%H-%M')
    df.to_csv(result_path+csv_name+now_fs+".csv", index=False)
    time_all=[]
    time_alg=[]
    for  seq in seq_list:
        try:
            file_name_alg = result_path+seq+"_time_cost_alg.csv"
            file_name_all = result_path+seq+"_time_cost.csv"
            result_files_to_folder.append(file_name_alg)
            result_files_to_folder.append(file_name_all)
            t_alg = np.loadtxt(file_name_alg,delimiter="\n")
            t_all = np.loadtxt(file_name_all,delimiter="\n")
            # os.remove(file_name_alg)
            # os.remove(file_name_all)
            if seq == "00":
                time_alg =t_alg
                time_all =t_all
            else:
                time_alg = np.hstack((time_alg,t_alg))
                time_all = np.hstack((time_all,t_all))
        except:pass
            
    # print(mean)
    print(time_all.shape)
    hz_alg = 1000/time_alg
    hz_all = 1000/time_all
    print(time_all.shape)
    
    time_cost_mean = np.stack([np.mean(time_all),np.mean(hz_all),np.mean(time_alg),np.mean(hz_alg)])
    time_cost_std      = np.stack([np.std(time_all, ddof=1),np.std(hz_all, ddof=1),np.std(time_alg, ddof=1),np.std(hz_alg, ddof=1)])
    time_cost = np.vstack((time_cost_mean, time_cost_std))
    df_time = pd.DataFrame(time_cost, columns=['time', 'hz', 'alg_time', 'alg_hz'])
    df_time.rename(index={0: 'mean',1: 'std'}, inplace=True)
    df_time.to_csv(result_path+ csv_name + '-' + now_fs+"-time_eval.csv", index=True)
    
    # create a folder named now_fs and move all the files in result_files_to_folder to this folder
    # create the folder in the result_path folder
    new_folder_name = result_path+now_fs
    os.mkdir(new_folder_name, 0o777)
    for file in result_files_to_folder:
        os.rename(file, new_folder_name+'/'+file.split('/')[-1])
    
    print("*"*10+"EVALUATION IS FINISHED"+"*"*10)

launch_path = sys.path[0] + '/../launch/eval_offline_kitti.launch'
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
cli_args = [launch_path,'seq:=04']
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
parent.start()

def handler(signum, frame):
    exit(1)
time_clock = time.time()
signal.signal(signal.SIGINT, handler)
one_batch_test()
print("finish all the test in:", time.time() - time_clock, "s")