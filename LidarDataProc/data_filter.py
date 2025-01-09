import json
from typing import Dict, List
import numpy as np

from LidarPointArray import LidarPointArray
from filter import *

def filter_lidar_data(lidar_data: List[LidarPointArray], filter_setting_path: str):
    """Filter the lidar data, according to a JSON filter file.
    Each point in the Points Cloud will be tested so it's not very optimised

    Args:
        lidar_data (List[LidarPointArray]): list of LidarPointArray to perform the filter on
        filter_setting_path (str): path to the filter file
    """

    print("Filtering the data with rules in {}".format(filter_setting_path))
    # Vars
    filter_obj: filter
    # Read File
    with open(filter_setting_path) as f_json:
        f_decode: Dict = json.loads(f_json.read())
        name = next(iter(f_decode))
        f_object = f_decode.get(name)
        # Set object
        if name == "range_filter":
            filter_obj = range_filter(**f_object)
    # Test filter for each point
    lpa: LidarPointArray
    # % compl
    i = 0.0
    length = len(lidar_data)
    print(" "*20, end='\r')
    percent: float = i / length * 100.0
    print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
    for lpa in lidar_data:
        # Affichage
        print(" "*20, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        i+=1
        # selection
        # set a as Narray
        #a: np.array = np.array(lpa.points_array)
        #a = np.array([ a[i,mask[i]] for i in xrange(a.shape[0]) ])
        #a = np.delete(a, np.argwhere(filter_obj.validate(np.array([0,0,0]), point=a)))
        #print(a)
        #exit()
        #lpa.points_array = a
        lpa.points_array = [p for p in lpa.points_array if filter_obj.validate(origine=np.array([0,0,0]), point=np.array(p))]
    print("Filtering finished!")
