from typing import List
from scipy import stats
import numpy as np

from WaveCluster import WaveCluster
from utils import calculate_distance

class WaveClusterTimelapse:
    """WaveClusterTimelapse Class, Represent the Cluster evolution through time

    Attributes : 
        wave_snapshots (List[WaveCluster]): list of WaveCluster

        slope (np.float): regression slop coef (a in the ax+b of a line)

        intercept (np.float): regression intercept coef (b in the ax+b of a line)

        rvalue (np.float): The Pearson correlation coefficient

        pvalue (np.float): The p-value for a hypothesis test whose null hypothesis is that the slope is zero, 
        using Wald Test with t-distribution of the test statistic (check scipy.stats.linregress doc for more info)

        angle (float): angle of the linear regression of the movement in degree (on 360 too by determining direction)

        finished (boolean): just a util var for an algo

        length_bary (float): distance from the first barypoint to the last (if big the cluster is probably significant if not it's just static)
    """
    def __init__(self, wave_snapshots) -> None:
        self.wave_snapshots: List[WaveCluster] = wave_snapshots
        self.slope, self.intercept, self.rvalue, self.pvalue = None, None, None, None
        self.angle = None
        self.finished = False
        self.length_bary = None

    def lin_regr(self):
        """Linear Regression Method

        Do the linear regression of the barycentre of each WaveCluster to determine the angle origine of the wave
        """
        wave: WaveCluster
        lx = [wave.barycentre[0] for wave in self.wave_snapshots]
        ly = [wave.barycentre[1] for wave in self.wave_snapshots]
        res = stats.linregress(lx, ly)
        self.slope, self.intercept, self.rvalue, self.pvalue = res.slope, res.intercept, res.rvalue, res.pvalue
        self.angle = np.rad2deg(np.arctan2(1 - -1, 1*self.slope - -1*self.slope))
        # increase presision for North or South
        NorS = self.angle < 45 or self.angle > 135
        if NorS:
            res = stats.linregress(ly, lx)
            angle = np.rad2deg(np.arctan2(1 - -1, 1*res.slope - -1*res.slope))
            if self.angle > 90:
                self.angle = 180 - (angle-90)
            else:
                self.angle = 90 - angle
        # test if angle is on the other side of the compase
        if self.angle < 45:
            if self.wave_snapshots[0].barycentre[1] < self.wave_snapshots[len(self.wave_snapshots)-1].barycentre[1]:
                self.angle = (self.angle+180)%360 # wave come from behind
        elif self.angle < 135:
            if self.wave_snapshots[0].barycentre[0] < self.wave_snapshots[len(self.wave_snapshots)-1].barycentre[0]:
                self.angle = (self.angle+180)%360 # wave come from behind
        else:
            if self.wave_snapshots[0].barycentre[1] > self.wave_snapshots[len(self.wave_snapshots)-1].barycentre[1]:
                self.angle = (self.angle+180)%360 # wave come from behind
        self.length_bary = calculate_distance(
            np.array(self.wave_snapshots[0].barycentre[0]), 
            np.array(self.wave_snapshots[len(self.wave_snapshots)-1].barycentre[0]))
