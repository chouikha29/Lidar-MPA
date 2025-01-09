from abc import ABC, abstractmethod
from typing import List
import numpy as np

from utils import *


# Abstract API
class filter(ABC):
    """Abstract Filter Class.

    Filter are used as .json file passed in arg to... filter data, this is very ineficient but I don't have time to improve it.

    A point must validate the... validate function to be included
    """
    # check if validate condition
    @abstractmethod
    def validate(self, origine: np.ndarray, point: np.ndarray) -> bool:
        pass


### OPERATOR ###
# AND filter operator
class filter_and(filter):
    """AND Filter Class.

    Will validate if all sub filters in the list_filter list are validated.
    """
    list_filter: List[filter]

    def validate(self, origine: np.ndarray, point: np.ndarray) -> bool:
        for f in self.list_filter:
            if not f.validate(origine, point):
                return False
        return True

# OR filter operator
class filter_or(filter):
    """OR Filter Class.

    Will validate if one of the sub filters in list_filter is validated.
    """
    list_filter: List[filter]

    def validate(self, origine: np.ndarray, point: np.ndarray) -> bool:
        for f in self.list_filter:
            if f.validate(origine, point):
                return True
        return False


### FILTER CLASS ###
class range_filter(filter):
    """Range Filter Class.

    Include or exclude point if in a certain range from an origine point.
    """
    def validate(self, origine: np.ndarray, point: np.ndarray) -> bool:
        # calculate dist
        dist = calculate_distance(origine, point)
        if dist < self.max and dist > self.min:
            return self.inclustion
        else:
            return not self.inclustion 
    def __init__(self, min, max, inclustion):
        self.min = min
        self.max = max
        self.inclustion = inclustion
