from typing import List
from matplotlib import pyplot as plt
from matplotlib import animation as anim

from LidarPointArray import LidarPointArray


def hex2dAnimates(array_cloud: List[LidarPointArray], save: bool=False):
    """Display 2D animation with pyplot hexbin

    Args:
        array_cloud (List[LidarPointArray]): Lidar Snapshot List
        save (bool): save the video or not
    """
    print("Start Animation Generation")
    fig = plt.figure()
    ims = []
    length = len(array_cloud)
    i: int = 0
    for array in array_cloud:
        i += 1
        # % compl
        print(" "*20, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        # create frame
        an = plt.hexbin(
            [p[0] for p in array.points_array],
            [p[1] for p in array.points_array],
            [p[2] for p in array.points_array]
        )
        ims.append([an])
    
    # lunch animation
    print("Lunch Animation")
    dt_interval = array_cloud[1].timestamp - array_cloud[0].timestamp
    interval = dt_interval.total_seconds() * 1000
    ani = anim.ArtistAnimation(fig, ims, interval=interval, blit=False,repeat_delay=0)
    
    if save:
        _save_anim(ani)
    
    plt.show()

def wave_line_anim(array_points, array_line, elipsed_time):
    """display 2d animation of the waves lines regression

    Args:
        array_points (_type_): waves clusters
        array_line (_type_): clusters regression
        elipsed_time (_type_): time between two frame
    """
    print("Start Animation Generation")
    fig = plt.figure()
    ims = []
    length = len(array_line)
    i: int = 0
    for lines in array_line:
        # % compl
        print(" "*20, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        # create frame
        frame = []
        for line in lines:
            frame.append(plt.plot([l[0] for l in line], [l[1] for l in line])[0])
        for cluster in array_points[i]:
            frame.append(plt.scatter([p[0] for p in cluster], [p[1] for p in cluster], alpha=0.3))
        ims.append(frame)
        i += 1
    
    # lunch animation
    print("Lunch Animation")
    dt_interval = elipsed_time
    interval = dt_interval.total_seconds() * 1000
    ani = anim.ArtistAnimation(fig, ims, interval=interval*1.5, blit=False,repeat_delay=5)
    plt.show()

def barycentre_anim(array_points, bary_points, elipsed_time):
    """Display 2d Animation of the moving barycentre

    Args:
        array_points (_type_): waves clusters
        bary_points (_type_): bary points
        elipsed_time (_type_): time between two frame
    """
    print("Start Animation Generation")
    fig = plt.figure()
    ims = []
    length = len(bary_points)
    i: int = 0
    for points in bary_points:
        # % compl
        print(" "*20, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        # create frame
        frame = []
        for point in points:
            frame.append(plt.scatter(point[0], point[1]))
        for cluster in array_points[i]:
            frame.append(plt.scatter([p[0] for p in cluster], [p[1] for p in cluster], alpha=0.1))
        ims.append(frame)
        i += 1
    
    # lunch animation
    print("Lunch Animation")
    dt_interval = elipsed_time
    interval = dt_interval.total_seconds() * 1000
    ani = anim.ArtistAnimation(fig, ims, interval=interval*1.5, blit=False,repeat_delay=5)
    plt.show()

# color change the longer a line is
color_line_wave = ['#000066', '#0000cc', '#3366ff', '#99ccff', '#ffcccc', '#ff9966', '#ff6600', '#ff3300', '#ff0000']

def barycentre_anim_plus_line_wave(array_points, bary_points, line_wave, elipsed_time):
    """same as barycentre_anim but with line following the barycentre of cluster, those line become more red the longer they are 

    Args:
        array_points (_type_): waves clusters
        bary_points (_type_): bary points
        line_wave (_type_): line bary points
        elipsed_time (_type_): time between two frame
    """
    print("Start Animation Generation")
    fig = plt.figure()
    ims = []
    length = len(bary_points)
    i: int = 0
    for points in bary_points:
        # % compl
        print(" "*20, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        # create frame
        frame = []
        for point in points:
            frame.append(plt.scatter(point[0], point[1]))
        for cluster in array_points[i]:
            frame.append(plt.scatter([p[0] for p in cluster], [p[1] for p in cluster], alpha=0.1))
        for line in line_wave[i]:
            color = color_line_wave[min(len(line), len(color_line_wave))-1]
            frame.append(plt.plot([l[0] for l in line], [l[1] for l in line], color=color)[0])
        ims.append(frame)
        i += 1
    
    # lunch animation
    print("Lunch Animation")
    dt_interval = elipsed_time
    interval = dt_interval.total_seconds() * 1000
    ani = anim.ArtistAnimation(fig, ims, interval=interval*1.5, blit=False,repeat_delay=5)
    plt.show()

def barycentre_anim_line_wave_compass(array_points, bary_points, line_wave, coefs, elipsed_time):
    # TODO : REPLACE MANUAL CALCUL BY WAVECLUSTER TIMELAPSE
    """barycentre_anim_plus_line_wave but with a line indicating the wave direction

    Args:
        array_points (_type_): _description_
        bary_points (_type_): _description_
        line_wave (_type_): _description_
        coefs (_type_): _description_
        elipsed_time (_type_): _description_
    """
    print("Start Animation Generation")
    fig = plt.figure()
    ims = []
    length = len(bary_points)
    i: int = 0
    for points in bary_points:
        # % compl
        print(" "*20, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        # create frame
        frame = []
        for point in points:
            frame.append(plt.scatter(point[0], point[1]))
        for cluster in array_points[i]:
            frame.append(plt.scatter([p[0] for p in cluster], [p[1] for p in cluster], alpha=0.1))
        for line in line_wave[i]:
            color = color_line_wave[min(len(line), len(color_line_wave))-1]
            frame.append(plt.plot([l[0] for l in line], [l[1] for l in line], color=color)[0])
        x = [-2, 2]
        y = [coefs[i][0]*-1, coefs[i][0]*1]
        frame.append(plt.plot(x, y, color='black')[0]) # compass
        ims.append(frame)
        i += 1
    
    # lunch animation
    print("Lunch Animation")
    dt_interval = elipsed_time
    interval = dt_interval.total_seconds() * 1000
    ani = anim.ArtistAnimation(fig, ims, interval=interval*1.5, blit=False,repeat_delay=5)
    plt.show()

def _save_anim(ani: anim.ArtistAnimation):
    # save animation
    print("save Animation")
    ffmpeg_dir = "C:/Users/xavier.lemen/Downloads/ffmpeg-master-latest-win64-gpl/bin/ffmpeg.exe"
    plt.rcParams['animation.ffmpeg_path'] = ffmpeg_dir
    f = r"H://Videos/waveCompass.mp4"
    FFwriter = anim.FFMpegWriter()
    ani.save(f, writer=FFwriter)