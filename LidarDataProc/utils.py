import numpy as np

"""UTIL FUNCTION THAT COULD HELP
"""

# calculate dist
def calculate_distance(n_point, n_other_point):
    """Calculate distance between two points

    Args:
        n_point (_type_): point A
        n_other_point (_type_): point B

    Returns:
        _type_: distance
    """
    squared_dist = np.sum((n_point-n_other_point)**2, axis=0)
    return np.sqrt(squared_dist)

def lerp(x, a, b):
    """just lerp

    Args:
        x (_type_): x
        a (_type_): a
        b (_type_): b

    Returns:
        _type_: result of lerp
    """
    return a + x * (b-a)

def moving_average(a, n=3):
    """moving average function

    Args:
        a (_type_): list value to move average on
        n (int, optional): n. Defaults to 3.

    Returns:
        _type_: a averaged
    """
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return np.concatenate((a[:n-1], ret[n - 1:] / n))

def mediane_angles(angles):
    """median of angles (FUNCTION FROM CHATGPT)

    Args:
        angles (List[float]): Angle list

    Returns:
        float: median of inputed angle
    """
    # Convertir tous les angles en radians pour faciliter les calculs trigonométriques
    angles_radians = [angle * (3.14159 / 180) for angle in angles]

    # Trier les angles
    sorted_angles = sorted(angles_radians)

    # Trouver l'angle médian en convertissant en degrés
    median_radian = sorted_angles[len(sorted_angles) // 2]
    median_degrees = median_radian * (180 / 3.14159)

    # Correction pour les angles négatifs
    if median_degrees < 0:
        median_degrees += 360

    return median_degrees