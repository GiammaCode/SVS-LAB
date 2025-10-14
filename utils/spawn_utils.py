import sys
from selectors import SelectSelector

import carla
import time
import math
import carla
import random


def spawn_vehicle(world, model='vehicle.audi.tt'):
    """
    Cerca un punto di spawn libero e spawna un veicolo.
    Ritorna l'attore del veicolo o None se non ci sono punti liberi.
    """
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(model)[0]

    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

    if vehicle:
        print(f"Veicolo '{model}' spawnato con successo.")
        vehicle.set_autopilot(False)
    else:
        print(
            f"Impossibile spawnare il veicolo '{model}'. Nessun punto di spawn valido trovato dopo diversi tentativi.")

    return vehicle


def spawn_camera(world, vehicle, camera_transform: carla.Transform = None, img_width=800, img_height=600, fov=110):
    """
    Crea un sensore fotocamera e lo attacca al veicolo in una posizione specifica.

    Args:
        world (carla.World): Il mondo del simulatore.
        vehicle (carla.Actor): Il veicolo a cui attaccare la fotocamera.
        camera_transform (carla.Transform, optional): La posizione e rotazione della fotocamera
                                                     rispetto al veicolo. Se None, usa una posizione di default.
        img_width (int, optional): Larghezza dell'immagine. Default a 800.
        img_height (int, optional): Altezza dell'immagine. Default a 600.
        fov (int, optional): Campo visivo in gradi. Default a 110.

    Returns:
        carla.Actor: L'attore della fotocamera spawnata, o None se fallisce.
    """
    blueprint_library = world.get_blueprint_library()
    camera_bp = blueprint_library.find('sensor.camera.rgb')

    # Impostiamo gli attributi della fotocamera
    camera_bp.set_attribute('image_size_x', f'{img_width}')
    # Corretto: ora usa img_height invece di img_width
    camera_bp.set_attribute('image_size_y', f'{img_height}')
    camera_bp.set_attribute('fov', f'{fov}')
    camera_bp.set_attribute('sensor_tick', '0.0')  # 0.0 per il massimo frame rate possibile

    # Se non viene fornita una posizione, usiamo una posizione di default
    if camera_transform is None:
        print("Nessuna posizione fornita, uso la posizione di default.")
        camera_transform = carla.Transform(
            carla.Location(x=-1.5, z=2.0),  # x: indietro, z: in alto
            carla.Rotation(pitch=-15)  # Inclinata leggermente verso il basso
        )

    # Spawniamo la fotocamera e la attacchiamo al veicolo
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    if camera:
        print(f"Fotocamera spawnata con successo in posizione: {camera_transform.location}")
    else:
        print("Impossibile spawnare la fotocamera.")

    return camera