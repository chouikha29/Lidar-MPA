from datetime import datetime
import math
from statistics import mean, median
from tokenize import Double
from typing import List, Tuple
from matplotlib import pyplot as plt
from matplotlib import animation as anim
from scipy.stats import circmean

from WaveClusterTimelapse import WaveClusterTimelapse
from WaveCluster import WaveCluster
from utils import moving_average, mediane_angles

def wave_height(timestamps, timelapses):
    """graph of cluster height through time

    Args:
        timestamps (_type_): _description_
        timelapses (_type_): _description_
    """
    data = []
    timeslapse: WaveClusterTimelapse
    for timeslapse in timelapses:
        line = []
        for wave in timeslapse.wave_snapshots:
            line.append((wave.timestamp, wave.barycentre[2]))
        alpha = abs(timeslapse.rvalue)/2 + (min(timeslapse.length_bary,5)/5)/2
        plt.plot([p[0] for p in line], [p[1] for p in line], color="black", alpha=alpha)
    plt.show()

def polar_angle(timestamps, timelapses):
    """polar angle of the wave direction

    Args:
        timestamps (_type_): _description_
        timelapses (_type_): _description_
    """
    ax = plt.axes(polar=True)
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    # lines angles
    angleslines = []
    for timeslapse in timelapses:
        line_time = []
        line_angle = []
        for wave in timeslapse.wave_snapshots:
            line_time.append(wave.timestamp)
            line_angle.append(timeslapse.angle)
        angleslines.append([line_time, line_angle, timeslapse.rvalue, timeslapse.length_bary])
    for line in angleslines:
        alpha = abs(line[2])/2 + (min(line[3],5)/5)/2
        plt.plot([math.radians(angle) for angle in line[1]], line[0], color="black", alpha=alpha)
    
    # moy angles
    moy: List[Tuple[Double, datetime]] = []
    med: List[Tuple[Double, datetime]] = []
    pondered_moy: List[Tuple[Double, datetime]] = []
    pondered_med: List[Tuple[Double, datetime]] = []
    for timestamp in timestamps:
        concerned_timelapses = []

        timeslapse: WaveClusterTimelapse
        for timeslapse in timelapses:
            wave_snap: WaveCluster
            for wave_snap in timeslapse.wave_snapshots:
                if wave_snap.timestamp == timestamp:
                    concerned_timelapses.append(timeslapse)
        if concerned_timelapses:
            # moy / med
            moy.append(mean([timeslapse.angle for timeslapse in concerned_timelapses]))
            med.append(median([timeslapse.angle for timeslapse in concerned_timelapses]))
            # pondered weight
            tot_angle = []
            for timeslapse in concerned_timelapses:
                weight = abs(timeslapse.rvalue)*5 + min(timeslapse.length_bary,5)
                weight = int(weight)
                for _ in range(weight):
                    tot_angle.append(timeslapse.angle)
            pondered_moy.append((circmean(tot_angle, high=360), timestamp))
            pondered_med.append((mediane_angles(tot_angle), timestamp))
    #plt.plot(timestamps, moy, label="mean")
    #plt.plot(timestamps, med, label="median")
    #plt.plot(timestamps, pondered_moy, label="pondered mean")
    pondered_med_timestamps = [value[1] for value in pondered_med]
    moving_average_med = moving_average([value[0] for value in pondered_med], 20)
    plt.plot([math.radians(angle[0]) for angle in pondered_med], pondered_med_timestamps, label="pondered median")
    plt.plot([math.radians(angle) for angle in moving_average_med], pondered_med_timestamps, label="average pondered median")
    plt.legend(loc='best')
    plt.show()

def _save_anim(ani: anim.ArtistAnimation):
    # save animation
    print("save Animation")
    ffmpeg_dir = "C:/Users/xavier.lemen/Downloads/ffmpeg-master-latest-win64-gpl/bin/ffmpeg.exe"
    plt.rcParams['animation.ffmpeg_path'] = ffmpeg_dir
    f = r"H://Videos/waveCompass.mp4"
    FFwriter = anim.FFMpegWriter()
    ani.save(f, writer=FFwriter)
