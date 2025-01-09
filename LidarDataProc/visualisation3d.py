from typing import List
import open3d as o3d

from LidarPointArray import LidarPointArray
from LidarPointArray import LidarPointArray

### ANIMATION VARS ###
i_frame_anim: int
old_i: int
movie: bool
len_array: int

### ANIMATION FUNCTION ###
def _key_MINUS_PRESSED(vis):
    global i_frame_anim
    if i_frame_anim>0:
        i_frame_anim-=1
def _key_PLUS_PRESSED(vis):
    global i_frame_anim
    if i_frame_anim<len_array-1:
        i_frame_anim+=1
def _key_DIVIDE_PRESSED(vis):
    global i_frame_anim
    global movie
    if i_frame_anim>=len_array:
        i_frame_anim-=1
    movie = not movie
def _key_MULTIPLY_PRESSED(vis):
    global i_frame_anim
    i_frame_anim = 0

def _key_input_param(vis):
    # /
    vis.register_key_callback(331, _key_DIVIDE_PRESSED)
    # *
    vis.register_key_callback(332, _key_MULTIPLY_PRESSED)
    # -
    vis.register_key_callback(333, _key_MINUS_PRESSED)
    # +
    vis.register_key_callback(334, _key_PLUS_PRESSED)

def display_anim_point_array(array_cloud: List[LidarPointArray]):
    global i_frame_anim
    global old_i
    global movie
    global len_array
    """Display points array in 3D animation\n
    The animation will run as fast as possible without notion of time between Lidar snapshot

    Args:
        array_cloud (List[LidarPointArray]): List of Lidar Snapshot to display
    """
    # create window
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(
        window_name="CloudPoint Visualizer",
        width=1000,
        height=500,
        left=500,
        top=500
    )

    # visu param
    opt = vis.get_render_option()
    opt.point_show_normal = False
    opt.mesh_show_back_face = True
    _key_input_param(vis)

    # load first frame
    geometry = o3d.geometry.PointCloud()
    i_frame_anim = 0
    geometry.points = o3d.utility.Vector3dVector(array_cloud[i_frame_anim].points_array)
    geometry.voxel_down_sample(1.0)
    #geometry.estimate_normals()
    #geometry.orient_normals_towards_camera_location()
    vis.add_geometry(geometry)

    # run sim
    keep_running = True
    movie = False
    old_i = -1
    len_array = len(array_cloud)
    while keep_running:
        if (not movie) and (i_frame_anim!=old_i):
            geometry.points = o3d.utility.Vector3dVector(array_cloud[i_frame_anim].points_array)
            geometry.voxel_down_sample(1.0)
            vis.update_geometry(geometry)
            old_i = i_frame_anim
        elif (movie) and i_frame_anim<len_array:
            geometry.points = o3d.utility.Vector3dVector(array_cloud[i_frame_anim].points_array)
            geometry.voxel_down_sample(1.0)
            vis.update_geometry(geometry)
            old_i = i_frame_anim
            i_frame_anim+=1
        keep_running = vis.poll_events()

    # escape key
    vis.destroy_window()

def display_anim_mesh(array_geo, array_cloud):
    global i_frame_anim
    global old_i
    global movie
    global len_array
    """Display points array in 3D animation\n
    The animation will run as fast as possible without notion of time between Lidar snapshot

    Args:
        array_cloud (List[LidarPointArray]): List of Lidar Snapshot to display
    """
    # create window
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(
        window_name="Mesh Anim Visualizer",
        width=1000,
        height=500,
        left=500,
        top=500
    )

    # visu param
    opt = vis.get_render_option()
    opt.point_show_normal = False
    opt.mesh_show_back_face = True
    _key_input_param(vis)

    # load first frame
    i_frame_anim = 0

    # mesh
    mesh_arr = array_geo[i_frame_anim]
    for m in mesh_arr:
        vis.add_geometry(m)
    # point cloud
    pc = array_cloud[i_frame_anim]
    vis.add_geometry(pc)

    # run sim
    keep_running = True
    movie = False
    old_i = -1
    len_array = len(array_geo)
    while keep_running:
        if (not movie) and (i_frame_anim!=old_i):
            # update mesh
            for m in mesh_arr:
                vis.remove_geometry(m, reset_bounding_box=False)
            mesh_arr = array_geo[i_frame_anim]
            for m in mesh_arr:
                vis.add_geometry(m, reset_bounding_box=False)
            # update point cloud
            pc.points = array_cloud[i_frame_anim].points
            vis.update_geometry(pc)
            old_i = i_frame_anim
        elif (movie) and i_frame_anim<len(array_geo):
            # update mesh
            for m in mesh_arr:
                vis.remove_geometry(m, reset_bounding_box=False)
            mesh_arr = array_geo[i_frame_anim]
            for m in mesh_arr:
                vis.add_geometry(m, reset_bounding_box=False)
            # update point cloud
            pc.points = array_cloud[i_frame_anim].points
            vis.update_geometry(pc)
            i_frame_anim+=1
        keep_running = vis.poll_events()
    
    # escape key
    vis.destroy_window()
