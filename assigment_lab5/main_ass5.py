import carla
import cv2
import numpy as np
import random
import time
import math

import utils.spawn_utils
import utils.sensor_utils

# --- Costanti di configurazione ---
HOST = 'localhost'
PORT = 2000


def main():
    actor_list = []
    radar_data = {'detections': [], 'min_ttc': float('inf')}

    try:
        # connect
        client = carla.Client(HOST, PORT)
        client.set_timeout(10.0)
        print("Connessione a CARLA...")
        world = client.get_world()
        print("Connessione riuscita!")

        spectator = world.get_spectator()

        # spawn actors with sensor
        ego_vehicle = utils.spawn_utils.spawn_vehicle(world, 'vehicle.audi.tt')
        if ego_vehicle is None:
            return
        actor_list.append(ego_vehicle)

        target_vehicle = utils.spawn_utils.spawn_vehicle(world, 'vehicle.volkswagen.t2')
        if target_vehicle is None:
            return
        actor_list.append(target_vehicle)

        ego_transform = ego_vehicle.get_transform()

        target_transform = carla.Transform(
            ego_transform.location + carla.Location(x=10, z=0),
            ego_transform.rotation
        )
        target_vehicle.set_transform(target_transform)

        spectator_location = ego_transform.transform(carla.Location(x=-8, z=3))
        spectator_transform = carla.Transform(spectator_location, ego_transform.rotation)
        spectator.set_transform(spectator_transform)

        # spawn sensor
        TTC_THRESHOLD = 2.0  # seconds. If TTC is less than this, apply emergency braking
        SENSOR_RANGE = 50.0  # meters. Maximum range of the radar

        # configure radar
        blueprint_library = world.get_blueprint_library()
        radar_bp = blueprint_library.find('sensor.other.radar')

        # Aumentiamo il campo visivo per una migliore copertura
        radar_bp.set_attribute('horizontal_fov', str(45))  # field of views degrees
        radar_bp.set_attribute('vertical_fov', str(30))  # Aumentato da 20 a 30
        radar_bp.set_attribute('points_per_second', '1500')
        radar_bp.set_attribute('range', f'{SENSOR_RANGE}')

        # Position the radar
        radar_transform = carla.Transform(carla.Location(x=2.5, z=1.0))
        radar_sensor = world.spawn_actor(radar_bp, radar_transform, attach_to=ego_vehicle)
        actor_list.append(radar_sensor)

        radar_sensor.listen(lambda data: utils.sensor_utils.radar_callback(data, radar_data, ego_vehicle))
        print("Radar sensor activated")

        # --- 4. CICLO PRINCIPALE DI CONTROLLO ---
        print("\nInizio del test di frenata di emergenza.")
        print(f"Il veicolo avanzerà lentamente. Se il TTC è inferiore a {TTC_THRESHOLD}s, i freni verranno applicati.")
        print("Premi Ctrl+C per terminare.")

        # Aggiungiamo un contatore per stabilizzare le letture
        detection_counter = 0
        stable_detection_threshold = 3  # Numero di rilevamenti consecutivi prima di considerare stabile

        while True:
            world.tick()

            ego_transform = ego_vehicle.get_transform()
            spectator_location = ego_transform.transform(carla.Location(x=-8, z=3))
            spectator.set_transform(carla.Transform(spectator_location, ego_transform.rotation))

            # Ottieni il TTC più recente calcolato dal callback
            current_ttc = radar_data['min_ttc']

            # Filtriamo i dati per evitare falsi positivi
            if current_ttc < TTC_THRESHOLD:
                detection_counter += 1
            else:
                detection_counter = 0

            # Logica di controllo con rilevamento stabile
            if detection_counter >= stable_detection_threshold:
                # OSTACOLO RILEVATO STABILE! APPLICA I FRENI
                control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                ego_vehicle.apply_control(control)
                print(f"OSTACOLO RILEVATO! TTC: {current_ttc:.2f}s! FRENATA ATTIVATA")
            else:
                # NESSUN OSTACOLO. AVANZA LENTAMENTE
                control = carla.VehicleControl(throttle=0.5, brake=0.0, steer=0.0)
                ego_vehicle.apply_control(control)

                if detection_counter > 0:
                    print(
                        f"Rilevamento possibile ({detection_counter}/{stable_detection_threshold}) - TTC: {current_ttc:.2f}s")
                else:
                    print(f"Nessun ostacolo rilevato - TTC: {current_ttc:.2f}s")

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