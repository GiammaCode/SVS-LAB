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
    image_data = {'image': None}  # Dizionario per condividere l'immagine tra il callback e il main loop

    all_camera_data = {'front': None, 'rear': None, 'left': None, 'right': None}  # Dizionario per condividere l'immagine tra il callback e il main loop
    actor_list = []  # Lista per tenere traccia di tutti gli attori creati (veicolo, sensori)

    front_location = carla.Location(x=1.5, y=0, z=1.8)  # x: avanti, y: centro, z: altezza
    front_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)  # dritta, rivolta in avanti
    front_transform = carla.Transform(front_location, front_rotation)

    rear_location = carla.Location(x=-1.5, y=0, z=1.8)  # x: indietro
    rear_rotation = carla.Rotation(pitch=0, yaw=180, roll=0)  # yaw di 180 gradi
    rear_transform = carla.Transform(rear_location, rear_rotation)

    left_location = carla.Location(x=-1.5, y=0, z=1.8)  # x: indietro
    left_rotation = carla.Rotation(pitch=0, yaw=270, roll=0)  # yaw di 180 gradi
    left_transform = carla.Transform(left_location, left_rotation)

    right_location = carla.Location(x=-1.5, y=0, z=1.8)  # x: indietro
    right_rotation = carla.Rotation(pitch=0, yaw=90, roll=0)  # yaw di 180 gradi
    right_transform = carla.Transform(right_location, right_rotation)


    try:
        #connect
        client= carla.Client(HOST, PORT)
        client.set_timeout(10.0)
        print("Connessione a CARLA...")
        world = client.get_world()
        print("Connessione riuscita!")

        #spawn actors
        vehicle = utils.spawn_utils.spawn_vehicle(world)
        if vehicle is None:
            return
        actor_list.append(vehicle)

        Frcamera = utils.spawn_utils.spawn_camera(world, vehicle, front_transform)
        Recamera = utils.spawn_utils.spawn_camera(world, vehicle, rear_transform)
        Lecamera = utils.spawn_utils.spawn_camera(world, vehicle, left_transform)
        Ricamera = utils.spawn_utils.spawn_camera(world, vehicle, right_transform)

        actor_list.append(Frcamera)
        actor_list.append(Recamera)
        actor_list.append(Lecamera)
        actor_list.append(Ricamera)

        #sensor
        # Avviamo il sensore. Ogni nuova immagine chiamerà la funzione 'camera_callback'

        Frcamera.listen(lambda image: utils.sensor_utils.camera_callback(image, all_camera_data, 'front'))
        Recamera.listen(lambda image: utils.sensor_utils.camera_callback(image, all_camera_data, 'rear'))
        Lecamera.listen(lambda image: utils.sensor_utils.camera_callback(image, all_camera_data, 'left'))
        Ricamera.listen(lambda image: utils.sensor_utils.camera_callback(image, all_camera_data, 'right'))

        print("Sensore fotocamera attivo. In attesa di immagini...")

        # Creiamo una finestra con OpenCV
        cv2.namedWindow('Front-Cam', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Rear-Cam', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Left-Cam', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Right-Cam', cv2.WINDOW_NORMAL)

        print("\nPremi 'q' sulla finestra della fotocamera per chiudere.")
        while True:
            # Avanza la simulazione. È FONDAMENTALE per generare nuovi dati dai sensori.
            world.tick()

            # Controlla se un'immagine è stata catturata dal callback
            if all_camera_data['front'] is not None:
                cv2.imshow('Front-Cam', all_camera_data['front'])
            if all_camera_data['rear'] is not None:
                cv2.imshow('Rear-Cam', all_camera_data['rear'])
            if all_camera_data['left'] is not None:
                cv2.imshow('Left-Cam', all_camera_data['left'])
            if all_camera_data['right'] is not None:
                cv2.imshow('Right-Cam', all_camera_data['right'])

            # Aspetta 1ms per un input da tastiera. Se 'q' è premuto, esci dal ciclo.
            if cv2.waitKey(1) == ord('q'):
                print("Tasto 'q' premuto. Chiusura in corso...")
                break

    except Exception as e:
        print(f"Si è verificato un errore: {e}")

    finally:
        # --- 5. PULIZIA DEGLI ATTORI ---
        # Questo blocco viene eseguito sempre, sia in caso di errore che di uscita normale.
        print("Pulizia degli attori...")
        cv2.destroyAllWindows()  # Chiude la finestra di OpenCV

        for actor in actor_list:
            if actor.is_alive:
                print(f"Distruggendo: {actor.type_id}")
                actor.destroy()

        print("Script terminato.")

if __name__ == '__main__':
        main()

