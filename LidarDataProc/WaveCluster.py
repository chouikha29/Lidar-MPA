import datetime
from statistics import median

class WaveCluster:
    def __init__(self, clusters_wave_points, timestamp) -> None:
        self.cluster = clusters_wave_points
        self.timestamp: datetime = timestamp
        self.barycentre = self._bar_cen_cluster_calc(self.cluster)

    def _bar_cen_cluster_calc(self, cluster):
        x = median(p[0] for p in cluster)
        y = median(p[1] for p in cluster)
        z = median(p[2] for p in cluster)
        return [x,y,z]
        