import open3d as o3d
import numpy as np

def create_random_point_cloud():
    # Crée un nuage de points aléatoires
    num_points = 1000
    points = np.random.rand(num_points, 3)  # Génère 1000 points aléatoires
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

def visualize_point_cloud(point_cloud):
    # Affiche le nuage de points
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(point_cloud)
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    # Génère un nuage de points aléatoires
    point_cloud = create_random_point_cloud()

    # Tente de visualiser les points en 3D
    visualize_point_cloud(point_cloud)
