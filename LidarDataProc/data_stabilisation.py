# IMPORT EXTERN
import math
from typing import List
import open3d as o3d
import numpy as np

from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
# IMPORT CLASS
from LidarPointArray import LidarPointArray
from GyroData import GyroData


def _rotate_around_point(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def _correct_array_point(array_lidar: LidarPointArray, tot_yaw: float, tot_pitch: float, tot_roll: float, stabil_param: str):
    """Private function to stabilise a LidarPointArray according to the inputed yaw, pitch and roll.

    For exemple if stabil_param = ypr, it will correct all axes, but if it's just y, it will only apply a yaw stabilisation

    Args:
        array_lidar (LidarPointArray): LidarPointArray to stabilise
        tot_yaw (float): yaw to correct
        tot_pitch (float): pitch to correct
        tot_roll (float): roll to correct
        stabil_param (string): stabilisation param inputed by the user (string of each axe to correct, ypr)

    Returns:
        LidarPointArray: stabilised LidarPointArray
    """
    # store new value
    lid = array_lidar

    for y in range(len(lid.points_array)):
        n_x = lid.points_array[y][0]
        n_y = lid.points_array[y][1]
        n_z = lid.points_array[y][2]
        # negate yaw
        if "y" in stabil_param.lower():
            n_x, n_y = _rotate_around_point((0,0), (lid.points_array[y][0], lid.points_array[y][1]), tot_yaw)
        # negate pitch
        if "p" in stabil_param.lower():
            n_y, n_z = _rotate_around_point((n_y,0), (n_y, lid.points_array[y][2]), tot_pitch)
        # negate roll
        if "r" in stabil_param.lower():
            n_x, n_z = _rotate_around_point((0,n_z), (n_x, n_z), tot_roll)
        lid.points_array[y][0] = n_x
        lid.points_array[y][1] = n_y
        lid.points_array[y][2] = n_z
    return lid

def stabilise_lidar_array(array_lidar: List[LidarPointArray], array_gyro: List[GyroData], stabil_param: str):
    """Stabilise a List of LidarPointArray with a List of GyroData and the param yaw/pitch/roll (ypr) to correct.

    The Stabilisation Algo is simple, it go through all GyroData and LidarPointArray at the same time.
    
    If a GyroData timestamp is greater than the current LidarPointArray, correct it with _correct_array_point, then test the next LidarPointArray.

    The Algo stop when there is no more GyroData or LidarPointArray to correct.

    It thus can correct a list of LidarPointArray even if the GyroData doesn't cover the entire lenght or if one is of higher frequency.

    Args:
        array_lidar (List[LidarPointArray]): lidar array point cloud
        array_gyro (List[GyroData]): gyro data array
        stabil_param (str): stabilidation param

    Returns:
        List[LidarPointArray]: corrected lidar array point cloud
    """
    print("Stabilising data of lenght : {}".format(len(array_lidar)))
    # array of corrected points
    new_array: List[LidarPointArray] = []
    
    # changed accel axes
    tot_yaw: float = 0.0
    tot_pitch: float = 0.0
    tot_roll: float = 0.0

    i: int = 0
    length = len(array_gyro)
    l_gyr: int = 0
    init_yaw_gyro = math.radians(float(array_gyro[0].yaw))
    init_pitch_gyro = math.radians(float(array_gyro[0].pitch))
    init_roll_gyro = math.radians(float(array_gyro[0].roll))

    # go through all gyro data
    for gyr in array_gyro:
        # % compl
        print(" "*20, end='\r')
        percent: float = l_gyr / length * 100.0
        print("{:.0f}/{} - {:.2f}%".format(l_gyr, length, percent), end='\r')
        l_gyr += 1

        if i >= len(array_lidar):
            # no more data to correct
            print("Breaked before end of Gyro Data")
            break
        while(i<len(array_lidar) and array_lidar[i].timestamp < gyr.timestamp):
            # data to correct
            lid = _correct_array_point(array_lidar[i], tot_yaw, tot_pitch, tot_roll, stabil_param)
            new_array.append(lid)
            i += 1
        # correct tot gyr
        tot_yaw = init_yaw_gyro-math.radians(float(gyr.yaw))
        tot_pitch = init_pitch_gyro-math.radians(float(gyr.pitch))
        tot_roll = init_roll_gyro-math.radians(float(gyr.roll))

    # if some lidar data left... use last ditch correction
    while(i<len(array_lidar)):
        # data to correct
        lid = _correct_array_point(array_lidar[i], tot_yaw, tot_pitch, tot_roll, stabil_param)
        new_array.append(lid)
        i += 1

    print("Finished stabilisation")
    return new_array
def detect_bassin(lidar_points: np.ndarray, distance_threshold=0.01, ransac_n=3, num_iterations=90000000):
    """Détecte un plan (le bassin) dans les données LiDAR à l'aide de RANSAC.

    Args:
        lidar_points (np.ndarray): Tableau des points 3D du nuage LiDAR.
        distance_threshold (float): Distance maximale pour qu'un point soit considéré comme appartenant au plan.
        ransac_n (int): Nombre minimum de points pour générer un modèle de plan.
        num_iterations (int): Nombre d'itérations de RANSAC.

    Returns:
        plane_model (tuple): Modèle du plan détecté (a, b, c, d) pour ax + by + cz + d = 0.
        inlier_points (np.ndarray): Points appartenant au plan.
        outlier_points (np.ndarray): Points restant après suppression des points du plan.
    """
    # Créer un nuage de points Open3D
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(lidar_points)

    # Appliquer RANSAC pour détecter un plan
    plane_model, inliers = point_cloud.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations
    )

    # Récupérer les points appartenant au plan et les points restants
    inlier_cloud = point_cloud.select_by_index(inliers)
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)

    return plane_model, np.asarray(inlier_cloud.points), np.asarray(outlier_cloud.points)
def detect_and_visualize_all_planes(lidar_points: np.ndarray, max_planes=10, distance_threshold=0.05, ransac_n=3, num_iterations=2000):
    """Détecte tous les plans dans les données LiDAR et comble les espaces vides.

    Args:
        lidar_points (np.ndarray): Tableau des points 3D du nuage LiDAR.
        max_planes (int): Nombre maximum de plans à détecter.
        distance_threshold (float): Distance maximale pour qu'un point soit considéré comme appartenant au plan.
        ransac_n (int): Nombre minimum de points pour générer un modèle de plan.
        num_iterations (int): Nombre d'itérations de RANSAC.

    Returns:
        geometries (list): Liste des nuages de points Open3D représentant chaque plan détecté.
    """
    planes = []
    all_inliers = []
    remaining_points = lidar_points  # Points restants pour la prochaine détection
    geometries = []

    # Étape 1 : Segmentation progressive
    for _ in range(max_planes):
        # Créer un nuage de points Open3D pour les points restants
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(remaining_points)

        # Appliquer RANSAC pour détecter un plan
        plane_model, inliers = point_cloud.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )

        # Si trop peu de points dans le plan, arrêter
        if len(inliers) < 100:  # Exemple : au moins 100 points nécessaires pour un plan valide
            break

        # Extraire les points du plan détecté
        inlier_cloud = point_cloud.select_by_index(inliers)
        inliers_array = np.asarray(inlier_cloud.points)

        # Ajouter le plan détecté
        planes.append(plane_model)
        all_inliers.append(inliers_array)

        # Supprimer les points du plan détecté des points restants
        outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
        remaining_points = np.asarray(outlier_cloud.points)

        # Ajouter les points du plan détecté à la visualisation
        inlier_cloud.paint_uniform_color(np.random.rand(3))  # Couleur aléatoire pour chaque plan
        geometries.append(inlier_cloud)

    # Étape 2 : Assigner les points restants au plan le plus proche
    if len(remaining_points) > 0:
        print(f"Points restants : {len(remaining_points)}. Assignation au plan le plus proche...")
        remaining_cloud = assign_points_to_nearest_plane(remaining_points, all_inliers)
        geometries.append(remaining_cloud)

    return geometries


def advanced_reconstruction(lidar_points, max_planes=10, distance_threshold=0.05, ransac_n=3, num_iterations=2000):
    """Reconstruit toutes les surfaces et minimise les espaces vides."""
    planes = []
    all_inliers = []
    remaining_points = lidar_points  # Points restants pour la prochaine détection
    geometries = []

    # Étape 1 : Segmentation progressive avec RANSAC
    for i in range(max_planes):
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(remaining_points)
        
        dynamic_distance_threshold = distance_threshold + (i * 0.01)  # Augmenter dynamiquement
        # Appliquer RANSAC pour détecter un plan
        plane_model, inliers = point_cloud.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )

        # Si trop peu de points dans le plan, arrêter
        if len(inliers) < 100:  # Par exemple, au moins 100 points nécessaires
            break

        # Extraire les points du plan détecté
        inlier_cloud = point_cloud.select_by_index(inliers)
        inliers_array = np.asarray(inlier_cloud.points)

        # Ajouter le plan détecté
        planes.append(plane_model)
        all_inliers.append(inliers_array)

        # Supprimer les points du plan détecté des points restants
        outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
        remaining_points = np.asarray(outlier_cloud.points)

        # Ajouter les points du plan détecté à la visualisation
        inlier_cloud.paint_uniform_color(np.random.rand(3))  # Couleur aléatoire pour chaque plan
        geometries.append(inlier_cloud)

        print(f"Plan {i+1} détecté avec {len(inliers)} points. Points restants : {len(remaining_points)}")

    # Étape 2 : Clustering des points restants
    if len(remaining_points) > 0:
        print("Clustering des points restants...")
        clusters = cluster_remaining_points(remaining_points)

        for cluster in clusters:
            cluster_cloud = o3d.geometry.PointCloud()
            cluster_cloud.points = o3d.utility.Vector3dVector(cluster)
            cluster_cloud.paint_uniform_color(np.random.rand(3))  # Couleur unique pour chaque cluster
            geometries.append(cluster_cloud)

    # Étape 3 : Assigner les points restants au plan le plus proche
    if len(remaining_points) > 0:
        print(f"Points restants après le clustering : {len(remaining_points)}. Assignation au plan le plus proche...")
        remaining_cloud = assign_points_to_nearest_plane(remaining_points, all_inliers)
        geometries.append(remaining_cloud)

    return geometries


def cluster_remaining_points(points, eps=0.5, min_samples=10):
    """Regroupe les points restants en clusters à l'aide de DBSCAN."""
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = clustering.labels_

    clusters = []
    for label in np.unique(labels):
        if label == -1:  # Bruit
            continue
        cluster = points[labels == label]
        clusters.append(cluster)
    return clusters


def assign_points_to_nearest_plane(points, all_planes_inliers):
    """Assigne les points restants au plan le plus proche pour éviter les espaces vides."""
    combined_inliers = np.vstack(all_planes_inliers)
    labels = []

    knn = NearestNeighbors(n_neighbors=1).fit(combined_inliers)
    distances, indices = knn.kneighbors(points)

    for idx in indices:
        labels.append(combined_inliers[idx[0]])

    assigned_points = np.array(labels).reshape(-1, 3)
    assigned_cloud = o3d.geometry.PointCloud()
    assigned_cloud.points = o3d.utility.Vector3dVector(assigned_points)
    assigned_cloud.paint_uniform_color([0.5, 0.5, 0.5])  # Couleur grise pour les points assignés

    return assigned_cloud


def reconstruct_surface_with_interpolation(lidar_points, alpha=0.05):
    """
    Complète les surfaces fragmentées par interpolation via une triangulation alpha shape.

    Args:
        lidar_points (np.ndarray): Points 3D du nuage LiDAR.
        alpha (float): Paramètre de l'alpha shape pour contrôler la densité de la triangulation.

    Returns:
        o3d.geometry.TriangleMesh: Maillage triangulé pour la reconstruction de surface.
    """
    print("Reconstruction des surfaces fragmentées avec interpolation...")

    # Étape 1 : Créer un nuage de points Open3D
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(lidar_points)

    # Filtrer les points pour éliminer le bruit
    point_cloud, _ = point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Étape 2 : Créer une surface triangulée avec Alpha Shape
    print("Génération de la surface triangulée...")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_cloud, alpha)
    mesh.compute_vertex_normals()

    return mesh