import open3d as o3d
import numpy as np
import time
import math
from data_stabilisation import detect_bassin

def animate_drone_movement(array_lidar, distance_threshold=0.05, ransac_n=3, num_iterations=2000):
    """
    Anime les trames LiDAR en affichant uniquement le drone et en fixant les murs.

    Args:
        array_lidar (List[LidarPointArray]): Trames LiDAR à traiter.
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
    vis.create_window("Animation du drone avec murs fixes")

    # Déterminer la position initiale du drone
    initial_points = np.array(array_lidar[0].points_array)
    plane_model, inliers, outliers = detect_bassin(
        initial_points, distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations
    )

    # Calcul du centre du bassin pour le référentiel
    bassin_center = np.mean(inliers, axis=0)
    reference_origin = bassin_center  # Utiliser uniquement le bassin comme référence

    # Fixer les murs
    murs_cloud = o3d.geometry.PointCloud()
    murs_cloud.points = o3d.utility.Vector3dVector(outliers - reference_origin)
    murs_cloud.paint_uniform_color([0.5, 0.5, 0.5])  # Murs en gris
    vis.add_geometry(murs_cloud)

    # Afficher les premiers points du drone avant l'animation
    print("Points initiaux du drone :")
    for p in initial_points[:5]:
        print(f"  {p}")

    # ---- Initialisation du drone ----
    drone_cloud = o3d.geometry.PointCloud()
    drone_cloud.points = o3d.utility.Vector3dVector(initial_points - reference_origin)  # Centrer le drone
    drone_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Drone en rouge
    vis.add_geometry(drone_cloud)

    # ---- Déplacement du drone ----
    drone_translation_step = 0.1  # Vitesse linéaire
    drone_amplitude = 2.0  # Amplitude du mouvement sinusoïdal
    drone_frequency = 0.1  # Fréquence du mouvement sinusoïdal

    # Animation sur 51 trames
    for frame_index, lidar_frame in enumerate(array_lidar[:51]):
        print(f"\n------ Trame {frame_index + 1}/51 ------")

        # Déplacer le drone uniquement
        x_movement = frame_index * drone_translation_step
        y_movement = drone_amplitude * math.sin(drone_frequency * frame_index)
        drone_translation = np.array([x_movement, y_movement, 0.0])

        # Mettre à jour le drone
        lidar_points = np.array(lidar_frame.points_array)
        adjusted_points = lidar_points - reference_origin + drone_translation  # Ajustement du mouvement
        drone_cloud.points = o3d.utility.Vector3dVector(adjusted_points)
        vis.update_geometry(drone_cloud)

        # 🔹 **Affichage des coordonnées après correction**
        print(f"Position du drone après déplacement : X={drone_translation[0]:.3f}, Y={drone_translation[1]:.3f}, Z={drone_translation[2]:.3f}")

        # Mettre à jour la visualisation
        vis.poll_events()
        vis.update_renderer()

        # Pause pour lisser l'animation
        time.sleep(0.1)

    # Fermer la visualisation une fois terminé
    vis.destroy_window()
    print("Animation terminée.")
