"""
LIDAR WATER SURFACE PROCESSING :

This Python Script allow to Process a LIDAR film scan of a water body's surface.

![wavedir](https://github.com/MalCaor/LidarWaterSurfaceProcessing/blob/master/img/wavedir.gif)

It can then display various informations about it, mainly the waves directions.

![polar](https://github.com/MalCaor/LidarWaterSurfaceProcessing/blob/master/img/polar.png)

If used as a module you can implement your own algorithm by using the various functions, mainly:

- file_parser: to parse a .pcap LIDAR file be it from a ouster or velodyne lidar.
- data_stabilisation: to stabilise the Lidar Point array with IMU data.
- line_generator: mainly for wave_clustering that enable to KNN the point cloud
- point_movement_line: process a clusterised array to follow the waves movement
"""