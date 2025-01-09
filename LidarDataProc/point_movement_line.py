from typing import List, Tuple
import numpy as np
from scipy import stats

from WaveCluster import WaveCluster
from WaveClusterTimelapse import WaveClusterTimelapse
from utils import calculate_distance

def wave_cluster_timelapse_generator(wave_clusters_frames):
    """Generate Wave Cluster Timelapse from Wave Cluster Frames

    Args:
        wave_clusters_frames (_type_): clusters

    Returns:
        List[WaveClusterTimelapse]: list of Wave Cluster Timelapse
    """
    length: float = len(wave_clusters_frames)
    list_wave_cluster_timelase_retour: List[WaveClusterTimelapse] = []
    
    i = 0
    print("linking clusters waves")
    for frame in wave_clusters_frames:
        # display
        print(" "*10, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        i+=1

        currtimelapses: List[WaveClusterTimelapse] = [timelapse for timelapse in list_wave_cluster_timelase_retour if timelapse.finished == False]
        if currtimelapses:
            # compare and continue if match
            timelapse: WaveClusterTimelapse
            for timelapse in currtimelapses:
                last_point: WaveCluster = timelapse.wave_snapshots[len(timelapse.wave_snapshots)-1]
                frame = sorted(frame, key=lambda elem: calculate_distance(np.array(last_point.barycentre), np.array(elem.barycentre)))
                if frame and calculate_distance(np.array(last_point.barycentre), frame[0].barycentre) < 1.5:
                    # continue line
                    timelapse.wave_snapshots.append(frame[0])
                    frame.remove(frame[0])
                else:
                    # line break, mark timelapse as finished
                    timelapse.finished = True
        # append frame
        for lone_point in frame:
            list_wave_cluster_timelase_retour.append(WaveClusterTimelapse([lone_point]))
    print("linking clusters waves Finished!")

    # remove useless timelapse
    list_wave_cluster_timelase_retour = [timelapse for timelapse in list_wave_cluster_timelase_retour if len(timelapse.wave_snapshots)>2]

    i = 0
    length = len(list_wave_cluster_timelase_retour)
    print("calculating line regression")
    timelapse: WaveClusterTimelapse
    for timelapse in list_wave_cluster_timelase_retour:
        # display
        print(" "*10, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        i+=1
        timelapse.lin_regr()
    print("line regression Finished!")

    print("wave_cluster_timelapse_generator finished")
    return list_wave_cluster_timelase_retour

def point_movement_line(barycentre_arrays):
    """generate lines from the movement of barycentres clusters

    Args:
        barycentre_arrays (_type_): array of barrycentres of clusters

    Returns:
        _type_: list of line
    """
    print("Interpreting array of length {}".format(str(len(barycentre_arrays))))
    list_line_frame = []

    for bc_point_cloud in barycentre_arrays:
        currline = []
        if list_line_frame:
            # load prev frame
            last_frame = list_line_frame[len(list_line_frame)-1]
            # compare and continue if match
            for line in last_frame:
                last_point = line[len(line)-1]
                bc_point_cloud = sorted(bc_point_cloud, key=lambda elem: calculate_distance(np.array(last_point), np.array(elem)))
                if bc_point_cloud and calculate_distance(np.array(last_point), bc_point_cloud[0]) < 1.5:
                    # continue line
                    l = list(line) # duplicate to not alter previous line
                    l.append(bc_point_cloud[0])
                    currline.append(l)
                    bc_point_cloud.remove(bc_point_cloud[0])
        # append frame
        for lone_point in bc_point_cloud:
            currline.append([lone_point])
        list_line_frame.append(currline)

    return list_line_frame

def find_direction_waves(list_lines):
    """Return a list coef of potential wave direction

    Args:
        list_lines (_type_): _description_

    Returns:
        _type_: list_coef_moy, list_coefs
    """
    print("Interpreting array of length {}".format(str(len(list_lines))))
    # slope, intercept, rvalue, pvalue
    list_coef_moy: List[Tuple[float, float, float, float]] = []
    list_coefs = []

    for lines in list_lines:
        frame_coef: List[Tuple[float, float, float, float]] = []
        for line in lines:
            if len(line)>2:
                lx = [p[0] for p in line]
                ly = [p[1] for p in line]
                res = stats.linregress(lx, ly)
                weight = abs(res.rvalue)*5 + len(line) # a clean and long line is more likely to be relevent
                weight = int(weight)
                for _ in range(weight):
                    frame_coef.append((res.slope, res.intercept, res.rvalue, res.pvalue))
        if frame_coef:
            list_coefs.append(frame_coef)
            tot_slope = sum(coef[0] for coef in frame_coef)
            moy_slope = tot_slope / len(frame_coef)
            tot_intercept = sum(coef[1] for coef in frame_coef)
            moy_intercept = tot_intercept / len(frame_coef)
            tot_rvalue = sum(coef[2] for coef in frame_coef)
            moy_rvalue = tot_rvalue / len(frame_coef)
            tot_pvalue = sum(coef[3] for coef in frame_coef)
            moy_pvalue = tot_pvalue / len(frame_coef)
            list_coef_moy.append((moy_slope, moy_intercept, moy_rvalue, moy_pvalue))
        else:
            list_coefs.append([(0.0, 0.0, 0.0, 0.0)])
            list_coef_moy.append((0.0, 0.0, 0.0, 0.0)) # default value
    
    return list_coef_moy, list_coefs