from typing import List

from GyroData import GyroData

def write_gyro_data(array_data: List[GyroData], path_file_output: str):
    """Legacy function to write GyroData in a file

    Args:
        array_data (List[GyroData]): GyroData array
        path_file_output (str): path of the file to write the data in (created if doesn't exist)
    """
    # write output
    print("Writing output in {}".format(path_file_output))
    f = open(path_file_output, "w")
    # mesure length
    length: float = len(array_data)
    i: float = 0.0
    # fo through array
    for data in array_data:
        # % compl
        print(" "*20, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        i += 1
        # write
        f.write(str(data)+"\n")

    print(" "*20, end='\r')
    print("Writing file {} Finished".format(path_file_output))
    f.close()
