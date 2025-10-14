import numpy as np
import math


# In utils/sensor_utils.py

def camera_callback(image, data_dict, camera_name):
    """
    Funzione di callback che salva l'immagine nel dizionario usando una chiave specifica.
    """
    # ... (codice per convertire l'immagine in un array numpy) ...
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]

    # Salva l'immagine nella chiave corretta
    data_dict[camera_name] = array


def lidar_callback(point_cloud, data_dict):
    """
    Funzione di callback per il sensore LIDAR.
    Elabora i dati del point cloud per trovare l'ostacolo più vicino.
    """
    # Converte i dati grezzi del LIDAR in un array NumPy
    # I dati sono una lista piatta di float [x1, y1, z1, i1, x2, y2, z2, i2, ...]
    points = np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))

    # Filtra i punti per considerare solo quelli davanti al veicolo (asse X > 0)
    # e quelli sopra il livello della strada (asse Z > -2.0, per escludere il terreno)
    front_points = points[(points[:, 0] > 0) & (points[:, 2] > -2.0)]

    if front_points.shape[0] > 0:
        # Calcola la distanza euclidea di ogni punto frontale dall'origine (il veicolo)
        # La distanza è sqrt(x^2 + y^2 + z^2)
        distances = np.linalg.norm(front_points[:, :3], axis=1)

        # Trova la distanza minima tra tutti i punti frontali
        min_distance = np.min(distances)
        data_dict['distance'] = min_distance
    else:
        # Nessun punto rilevato davanti, impostiamo la distanza a infinito
        data_dict['distance'] = float('inf')


def radar_callback(radar_data, data_dict, ego_vehicle):
    """
    Callback function for the radar sensor.
    Processes radar data to calculate Time to Collision (TTC).
    """
    # Get ego vehicle velocity
    ego_velocity = ego_vehicle.get_velocity()
    ego_speed = math.sqrt(ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2)

    # Process radar detections
    detections = []
    min_ttc = float('inf')

    # Pre-filter detections for efficiency
    for detection in radar_data:
        # Only consider detections in front of the vehicle
        if detection.azimuth < -90 or detection.azimuth > 90:
            continue

        # Get detection information
        distance = detection.depth
        relative_velocity = detection.velocity

        # Calculate closing speed (positive when approaching)
        closing_speed = -relative_velocity  # Negative because radar velocity is positive when approaching

        if closing_speed > 0.1:  # Only calculate TTC if objects are approaching significantly
            ttc = distance / closing_speed
            min_ttc = min(min_ttc, ttc)

            # Store detection information
            detections.append({
                'distance': distance,
                'relative_velocity': relative_velocity,
                'ttc': ttc,
                'azimuth': detection.azimuth
            })

    # Update the data dictionary
    data_dict['detections'] = detections
    data_dict['min_ttc'] = min_ttc