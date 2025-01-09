# IMPORT EXTERN
from typing import List
import open3d as o3d
import numpy as np

# IMPORT CLASS
from LidarPointArray import LidarPointArray
from utils import *

# param mesh
voxel_size = 0.1
dist_to_divide = 5
alpha = 1

# param line
dist_to_divide_line = 5


def shape_interpr(array_lidar: List[LidarPointArray]):
    """Return the shape tuple of list of mesh and point cloud from a list of lidar point array.

    This function is a Legacy feature that isn't really usefull anymore.

    The goal was to map the sea as a mesh to then use the numurous existing solution to get data from it.

    Args:
        array_lidar (List[LidarPointArray]): inputed list of lidar data

    Returns:
        Tuple[List[List[o3d.geometry]], List[o3d.geometry.PointCloud]]: tuple of list
    """
    length: float = len(array_lidar)
    print("Interpreting array of length {}".format(str(length)))
    list_mesh_retour: List[List[o3d.geometry.TriangleMesh]] = []
    list_pc_retour: List[o3d.geometry.PointCloud] = []
    i = 0.0
    for arr in array_lidar:        
        # percent
        print(" "*20, end='\r')
        percent: float = i / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(i, length, percent), end='\r')
        i += 1
        # calcuate shape instant
        _shape_arr(arr, list_pc_retour, list_mesh_retour)
    return (list_mesh_retour, list_pc_retour)

def _shape_arr(arr, list_pc_retour, list_mesh_retour):
    """Append a mesh and point cloud list of a single lidar "frame".

    The mesh is generated from a voxel downed version of the inputed point cloud.

    Mesh generation param are in the top file.

    TODO: PUT THOSE PARAMS IN A CONFIG.PY FILE (not enough time to do it, sorry)

    Args:
        arr (LidarPointArray): _description_
        list_pc_retour (List[o3d.geometry.PointCloud]): _description_
        list_mesh_retour ( List[List[o3d.geometry]): _description_
    """
    # create point cloud
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(arr.points_array)
    #pc.estimate_normals()
    #pc.orient_normals_towards_camera_location()
    pc = pc.voxel_down_sample(voxel_size=voxel_size)
    pc.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pc.remove_radius_outlier(nb_points=16, radius=0.05)
    list_pc_retour.append(pc)
    # GeneMesh from Cloud
    list_mesh_retour.append([_mesh_from_pc(pc)])

def _mesh_from_pc(point_coud):
    """Generate a mesh from a point cloud with o3d.geometry.TetraMesh.create_from_point_cloud.

    Args:
        pc_raw (List[List]): input point cloud

    Returns:
        o3d.geometry.TriangleMesh: mesh generated
    """
    tetra_mesh, pt_map =  o3d.geometry.TetraMesh.create_from_point_cloud(point_coud)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
        point_coud, alpha=alpha, tetra_mesh=tetra_mesh, pt_map=pt_map)
    mesh.compute_vertex_normals()
    return mesh

