import copy
import datetime
from random import random
from typing import List
import numpy as np

from LidarPointArray import LidarPointArray

class SimulatedSea:
    """Class to simulate a Fake Sea for testing purpose
    """
    def __init__(self, type, nbr_frames, intervals=0.1, start=datetime.datetime.today()) -> None:
        self.type = type
        self.nbr_frames = int(nbr_frames)
        self.intervals = intervals
        self.start: datetime = start

    def get_array_lidar(self):
        array_retour: List[LidarPointArray] = []
        waves = self._generate_waves_base(10)
        for i in range(self.nbr_frames):
            waves = copy.deepcopy(waves) # copy frame from origine
            for wave in waves:
                self._move_points(wave, 1, self._get_movement_by_type(self.type, i))
            pc = np.concatenate(waves)
            stamp: datetime = self.start + datetime.timedelta(0,self.intervals*i)
            frame: LidarPointArray = LidarPointArray(stamp.timestamp(), pc)
            array_retour.append(frame)
        return array_retour

    def _get_movement_by_type(self, type, i):
        x = 0
        y = 0
        z = 0
        # complex
        if type == "rotation":
            # do a 360 rotation for the entire seq
            angle = i/self.nbr_frames*360
            if  angle >= 0 and angle < 90:
                x = angle/90*-1
                y = (90-angle)/90*-1
            elif  angle >= 90 and angle < 180:
                x = (180-angle)/90*-1
                y = (angle-90)/90
            elif  angle >= 180 and angle < 270:
                x = (angle-180)/180
                y = (270-angle)/90
            elif  angle >= 270 and angle <= 360:
                x = (360-angle)/90
                y = (angle-270)/90*-1
            return [x,y,z]
        if type == "speen":
            # do speens
            angle = (i*2)%360
            if  angle >= 0 and angle < 90:
                x = angle/90*-1
                y = (90-angle)/90*-1
            elif  angle >= 90 and angle < 180:
                x = (180-angle)/90*-1
                y = (angle-90)/90
            elif  angle >= 180 and angle < 270:
                x = (angle-180)/180
                y = (270-angle)/90
            elif  angle >= 270 and angle <= 360:
                x = (360-angle)/90
                y = (angle-270)/90*-1
            return [x,y,z]

        # Simple
        if "n" in type:
            y=-1
        elif "s" in type:
            y=1
        if "e" in type:
            x=-1
        elif "w" in type:
            x=1
        return [x,y,z]

    def _generate_waves_base(self, nbr_waves):
        array_waves = []
        for _ in range(nbr_waves):
            pc = np.random.rand(int(random()*40+10),3)
            randx = (random()-0.5)*2
            randy = (random()-0.5)*2
            randz = (random()-0.5)*2
            self._move_points(pc, 1, [randx*nbr_waves,randy*nbr_waves,randz])
            array_waves.append(pc)
        return array_waves

    def _move_points(self, pc, speed, xyz):
        for i in range(len(pc)):
            rand = (random()-0.5)*2
            point = pc[i]
            point[0] = point[0]+(rand+speed*xyz[0])
            point[1] = point[1]+(rand+speed*xyz[1])
            point[2] = point[2]+(rand+speed*xyz[2])
            pc[i] = point
