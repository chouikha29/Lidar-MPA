import open3d as o3d
import numpy as np
import time
import math
from data_stabilisation import detect_bassin


def simulate_waves(points, frame_index, wave_amplitude=0.3, wave_frequency=0.2):
    """
    Simule des vagues en appliquant un déplacement sinusoïdal sur l'axe Z des points.

    Args:
        points (np.ndarray): Points 3D de la trame LiDAR.
        frame_index (int): Indice de la trame actuelle (pour simuler les vagues dynamiques).
        wave_amplitude (float): Amplitude des vagues.
        wave_frequency (float): Fréquence des vagues.

    Returns:
        np.ndarray: Points 3D modifiés avec l'effet des vagues.
    """
    points_with_waves = points.copy()
    for i, point in enumerate(points_with_waves):
        wave_offset = wave_amplitude * math.sin(wave_frequency * point[0] + frame_index * 0.2)
        points_with_waves[i, 2] += wave_offset
    return points_with_waves


def animate_drone_movement(array_lidar, max_planes=15, distance_threshold=0.05, ransac_n=3, num_iterations=2000):
    """
    Anime les trames LiDAR avec un drone en mouvement parcourant tout le bassin, avec des effets de vagues.

    Args:
        array_lidar (List[LidarPointArray]): Trames LiDAR à traiter.
        max_planes (int): Nombre maximum de plans à détecter.
        distance_threshold (float): Distance pour la détection des plans.
        ransac_n (int): Nombre minimum de points pour RANSAC.
        num_iterations (int): Nombre d'itérations pour RANSAC.
    """
    if not array_lidar or len(array_lidar) == 0:
        print("ERREUR : Aucun point LiDAR disponible pour l'animation.")
        return

    print(f"Nombre de trames LiDAR chargées : {len(array_lidar)}")

    # Initialiser la visualisation Open3D
    vis = o3d.visualization.Visualizer()
    vis.create_window("Animation dynamique des trames et du drone")

    # Fixer le référentiel sur le bassin
    initial_points = np.array(array_lidar[0].points_array)
    plane_model, inliers, outliers = detect_bassin(
        initial_points, distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations
    )

    # Bassin (points fixes)
    bassin_cloud = o3d.geometry.PointCloud()
    bassin_cloud.points = o3d.utility.Vector3dVector(inliers)
    bassin_cloud.paint_uniform_color([0.0, 1.0, 0.0])  # Bassin en vert
    vis.add_geometry(bassin_cloud)

    # Murs dynamiques avec vagues
    murs_cloud = o3d.geometry.PointCloud()
    murs_cloud.points = o3d.utility.Vector3dVector(outliers)
    murs_cloud.paint_uniform_color([0.5, 0.5, 0.5])  # Murs en gris
    vis.add_geometry(murs_cloud)

    # Drone
    drone_cloud = o3d.geometry.PointCloud()
    drone_cloud.points = o3d.utility.Vector3dVector(initial_points)
    drone_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Drone en rouge
    vis.add_geometry(drone_cloud)

    # Déplacement sinusoïdal du drone pour parcourir le bassin
    drone_translation = np.array([0.0, 0.0, 0.0])  # Position initiale
    drone_translation_step = 0.1  # Vitesse de déplacement linéaire
    drone_amplitude = 2.0  # Amplitude du mouvement sinusoïdal
    drone_frequency = 0.1  # Fréquence du mouvement sinusoïdal

    # Animation sur 51 trames
    for frame_index, lidar_frame in enumerate(array_lidar[:51]):
        print(f"Affichage de la trame {frame_index + 1}/51")

        # Simuler les vagues et mouvement dynamique sur les murs
        lidar_points = np.array(lidar_frame.points_array)
        murs_points = simulate_waves(outliers, frame_index)
        murs_cloud.points = o3d.utility.Vector3dVector(murs_points)
        vis.update_geometry(murs_cloud)

        # Déplacer le drone avec un mouvement sinusoïdal
        x_movement = frame_index * drone_translation_step
        y_movement = drone_amplitude * math.sin(drone_frequency * frame_index)
        drone_translation = np.array([x_movement, y_movement, 0.0])

        # Mettre à jour le drone
        lidar_points = np.array(lidar_frame.points_array)
        drone_cloud.points = o3d.utility.Vector3dVector(lidar_points)
        drone_cloud.translate(drone_translation, relative=False)  # Déplacement absolu
        vis.update_geometry(drone_cloud)

        # Mettre à jour la fenêtre de visualisation
        vis.poll_events()
        vis.update_renderer()

        # Pause pour lisser l'animation
        time.sleep(0.1)

    # Fermer la visualisation une fois terminé
    vis.destroy_window()
    print("Animation terminée.")
