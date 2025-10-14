# LIDAR (Light Detection and Ranging) is a method for determining ranges by targeting an object or a surface with a
# laser and measuring the time for the reflected light to return to the receiver.
#
# LIDAR sensors are used in autonomous vehicles to detect obstacles and create a 3D representation of the environment.
#
# Implment a script that detect obstacles in front of the vehicle and apply the brakes if the distance is less than a
# threshold, using the LIDAR sensor.
#
# Some notes on the simulation environment to test the emergency braking system:
#
# - Spawn a vehicle (i.e ego vehicle)
# - Spawn a vehicle (i.e target vehicle) an position it in front of the ego vehicle
# - Attach a LIDAR sensor to the ego vehicle
# - Create the logic to detect obstacles in front of the ego vehicle, based on the LIDAR sensor data.
# You can find the documentation for the LIDAR sensor [here](https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor).
# - Apply the brakes if the distance is less than a threshold.

import carla
import cv2
import numpy as np
import random
import time

import utils.spawn_utils
import utils.sensor_utils


# --- Costanti di configurazione ---
HOST = 'localhost'
PORT = 2000

def main():

    actor_list = []  # Lista per tenere traccia di tutti gli attori creati (veicolo, sensori)
    lidar_data = {'distance' : float('inf')}


    try:
        #connect
        client= carla.Client(HOST, PORT)
        client.set_timeout(10.0)
        print("Connessione a CARLA...")
        world = client.get_world()
        print("Connessione riuscita!")

        spectator = world.get_spectator()

        #spawn actors with sensor
        ego_vehicle = utils.spawn_utils.spawn_vehicle(world, 'vehicle.audi.tt')
        if ego_vehicle is None:
            return
        actor_list.append(ego_vehicle)


        target_vehicle = utils.spawn_utils.spawn_vehicle(world, 'vehicle.volkswagen.t2')
        if target_vehicle is None:
            return
        actor_list.append(target_vehicle)

        ego_transform = ego_vehicle.get_transform()

        spectator_location = ego_transform.transform(carla.Location(x=-8, z=3))
        spectator_transform = carla.Transform(spectator_location, ego_transform.rotation)
        spectator.set_transform(spectator_transform)

        target_transform = carla.Transform(
            ego_transform.location + carla.Location(x = 10 , z= 0),
            ego_transform.rotation
        )
        target_vehicle.set_transform(target_transform)

        #spawn sensor
        BRAKE_THRESHOLD = 5.0  # Metri. Se un ostacolo è più vicino, si frena.
        SENSOR_RANGE = 20.0  # Metri. Portata massima del LIDAR.

        blueprint_library = world.get_blueprint_library()
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

        # Configura gli attributi del LIDAR
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('points_per_second', '90000')
        lidar_bp.set_attribute('rotation_frequency', '10')
        lidar_bp.set_attribute('range', f'{SENSOR_RANGE}')

        # Posiziona il LIDAR sul tetto del veicolo ego
        lidar_transform = carla.Transform(carla.Location(z=2.5))
        lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        actor_list.append(lidar_sensor)

        # Avvia il sensore con il callback
        lidar_sensor.listen(lambda data: utils.sensor_utils.lidar_callback(data, lidar_data))
        print("Sensore LIDAR attivo.")
        # --- 4. CICLO PRINCIPALE DI CONTROLLO ---
        print("\nInizio del test di frenata di emergenza.")
        print(
            f"Il veicolo avanzerà lentamente. Se un ostacolo è a meno di {BRAKE_THRESHOLD}m, i freni verranno applicati.")
        print("Premi Ctrl+C per terminare.")
        while True:
            world.tick()

            ego_transform = ego_vehicle.get_transform()
            spectator_location = ego_transform.transform(carla.Location(x=-8, z=3))
            spectator.set_transform(carla.Transform(spectator_location, ego_transform.rotation))

            # Ottieni la distanza più recente calcolata dal callback
            current_distance = lidar_data['distance']

            # Logica di controllo
            if current_distance < BRAKE_THRESHOLD:
                # OSTACOLO RILEVATO! APPLICA I FRENI
                control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                ego_vehicle.apply_control(control)
                print(f"OSTACOLO RILEVATO a {current_distance:.2f}m! EMERGENCY BRAKE!", end='\r')
            else:
                # NESSUN OSTACOLO. AVANZA LENTAMENTE
                control = carla.VehicleControl(throttle=0.3, brake=0.0, steer=0.0)
                ego_vehicle.apply_control(control)
                print(f"Nessun ostacolo. Distanza minima: {current_distance:.2f}m. Avanzamento...", end='\r')

            # Piccola pausa per non sovraccaricare la CPU
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nScript interrotto dall'utente.")
    except Exception as e:
        print(f"error: {e}")

    finally:
        # --- 5. PULIZIA DEGLI ATTORI ---
        # Questo blocco viene eseguito sempre, sia in caso di errore che di uscita normale.
        print("Pulizia degli attori...")
        for actor in actor_list:
            if actor.is_alive:
                print(f"Distruggendo: {actor.type_id}")
                actor.destroy()

        print("Script terminato.")

if __name__ == '__main__':
        main()

