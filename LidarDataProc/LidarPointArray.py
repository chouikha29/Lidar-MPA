import datetime
import numpy as np


class LidarPointArray:
    """LidarPointArray Class representing a LIDAR 'frame'

    Attributes : 
        timestamp (datetime): datetime timestamp of the 'frame'

        points_array (List[List[]]): point cloud
    """

    def __init__(self, stamp: float, points: np.ndarray) -> None:
        self.timestamp: datetime = datetime.datetime.fromtimestamp(stamp)
        self.points_array = [[p[0],p[1],p[2]] for p in points]
