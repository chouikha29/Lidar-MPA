# Module Python Expérimental

## Reconstruction de Trajectoire et Simulation de Mouvement de Drone à partir de Données LiDAR

Reconstruction de Trajectoire et Simulation de Mouvement de Drone à partir de Données LiDAR

## Options disponibles :

```
  -h, --help            Affiche ce message d'aide et quitte.
  --lidar_vel CHEMIN_FICHIER_LIDAR NOMBRE_IMAGES_A_EXTRAIRE
                        Lit un fichier .pcap issu d'un Lidar Velodyne.
  --lidar_ous CHEMIN_FICHIER_LIDAR CHEMIN_FICHIER_JSON_META NOMBRE_IMAGES_A_EXTRAIRE
                        Lit un fichier .pcap issu d'un Lidar Ouster.
  --simu TYPE_MER NOMBRE_IMAGES
                        Génère une simulation de surface maritime.
  --gyro CHEMIN_FICHIER_CSV
                        Lit un fichier CSV contenant des données IMU.
  --corr OPTION_YPR
                        Corrige le nuage de points avec les données IMU (Yaw, Pitch, Roll).
  --prefilter CHEMIN_FICHIER_JSON
                        Filtre le nuage de points avant correction.
  --postfilter CHEMIN_FICHIER_JSON
                        Filtre le nuage de points après correction.
  --display TYPE_AFFICHAGE
                        Affiche les données sous différentes formes :
                        - pc (nuage de points)
                        - mesh (génération de maillage)
                        - hex2d (vue en hexagones 2D)
                        - barycentre (barycentre de la clusterisation KNN du nuage de points)
                        - linebary (mouvement du barycentre)
                        - wavedir (direction estimée des vagues)
                        - wavepolar (polarisation de la direction estimée des vagues)
                        - waveheight (hauteur moyenne de chaque cluster)
```

## Guide d'installation

### Prérequis

Assurez-vous d'avoir Python 3.10.4 ou une version supérieure.

### Installation

1. **Cloner le dépôt GitHub** :

   ```bash
   git clone https://github.com/chouikha29/Lidar-MPA
   ```

2. **Accéder au dossier du projet** :

   ```bash
   cd Lidar-MPA
   ```

3. **Installer les dépendances** :

   ```bash
   pip install .
   ```

   **OU** utiliser l'environnement Conda :

   ```bash
   conda env create --file=Conda/env.yaml
   ```

### Vérification de l'installation

Tester si tout fonctionne en exécutant :

```bash
python ./LidarDataProc/LidarDataProc.py --help
```

Si le message d'aide s'affiche (cela peut prendre quelques secondes pour importer les modules), félicitations ! 🎉

### Lancer l'animation

Pour exécuter l'animation, utilisez la commande suivante :

```bash
python ./LidarDataProc/LidarDataProc.py --lidar_vel ./Lidar/Data1.pcap 51 --display animation
```

