import carla
import cv2  # Libreria per la visualizzazione delle immagini
import numpy as np  # Libreria per la gestione degli array numerici (le immagini sono array)
import random
import time

# --- Costanti di configurazione ---
HOST = 'localhost'
PORT = 2000
IMG_WIDTH = 800
IMG_HEIGHT = 600
FOV = 110  # Campo visivo della fotocamera in gradi


def spawn_vehicle(world):
    """
    Sceglie un veicolo a caso e lo spawn in un punto di spawn valido.
    Ritorna l'attore del veicolo o None se lo spawn fallisce.
    """
    blueprint_library = world.get_blueprint_library()
    # Scegliamo un veicolo specifico per semplicità, ma potremmo sceglierne uno a caso
    vehicle_bp = blueprint_library.filter('vehicle.audi.tt')[0]

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("Errore: Nessun punto di spawn trovato sulla mappa.")
        return None

    spawn_point = random.choice(spawn_points)
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

    if vehicle:
        print(f"Veicolo spawnato con successo: {vehicle.type_id}")
        # Disattiviamo l'autopilota per tenerlo fermo
        vehicle.set_autopilot(False)
    else:
        print("Impossibile spawnare il veicolo.")

    return vehicle


def spawn_camera(world, vehicle):
    """
    Crea un sensore fotocamera e lo attacca al veicolo.
    Ritorna l'attore della fotocamera.
    """
    blueprint_library = world.get_blueprint_library()
    camera_bp = blueprint_library.find('sensor.camera.rgb')

    # Impostiamo gli attributi della fotocamera
    camera_bp.set_attribute('image_size_x', f'{IMG_WIDTH}')
    camera_bp.set_attribute('image_size_y', f'{IMG_HEIGHT}')
    camera_bp.set_attribute('fov', f'{FOV}')
    camera_bp.set_attribute('sensor_tick', '0.0')  # 0.0 per il massimo frame rate possibile

    # Definiamo la posizione della fotocamera rispetto al veicolo
    # Posizionata sopra e leggermente dietro il veicolo, rivolta in avanti
    camera_transform = carla.Transform(
        carla.Location(x=-1.5, z=2.0),  # x: indietro, z: in alto
        carla.Rotation(pitch=-15)  # Inclinata leggermente verso il basso
    )

    # Spawniamo la fotocamera e la attacchiamo al veicolo
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    if camera:
        print("Fotocamera spawnata e attaccata al veicolo.")
    else:
        print("Impossibile spawnare la fotocamera.")

    return camera


def camera_callback(image, data_dict):
    """
    Funzione di callback eseguita ogni volta che la fotocamera cattura un'immagine.
    Converte l'immagine grezza di CARLA in un array NumPy utilizzabile da OpenCV.
    """
    # 1. Converte i dati grezzi in un array NumPy di uint8
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))

    # 2. Rimodella l'array in una matrice 3D (Altezza, Larghezza, Canali Colore)
    # CARLA fornisce i dati in formato BGRA (Blue, Green, Red, Alpha), quindi 4 canali
    array = np.reshape(array, (image.height, image.width, 4))

    # 3. Rimuove il canale Alpha (trasparenza) per ottenere un'immagine a 3 canali (BGR)
    array = array[:, :, :3]

    # 4. Converte l'immagine da BGR a RGB, che è lo standard per OpenCV imshow
    array = array[:, :, ::-1]

    # 5. Salva l'immagine elaborata nel dizionario condiviso
    data_dict['image'] = array


def main():
    """
    Funzione principale per avviare il test della fotocamera.
    """
    actor_list = []  # Lista per tenere traccia di tutti gli attori creati (veicolo, sensori)
    image_data = {'image': None}  # Dizionario per condividere l'immagine tra il callback e il main loop

    try:
        # --- 1. CONNESSIONE A CARLA ---
        client = carla.Client(HOST, PORT)
        client.set_timeout(10.0)
        print("Connessione a CARLA...")
        world = client.get_world()
        print("Connessione riuscita!")

        # --- 2. SPAWN DEGLI ATTORI ---
        vehicle = spawn_vehicle(world)
        if vehicle is None:
            return
        actor_list.append(vehicle)

        camera = spawn_camera(world, vehicle)
        if camera is None:
            return
        actor_list.append(camera)

        # --- 3. AVVIO DEL SENSORE ---
        # Avviamo il sensore. Ogni nuova immagine chiamerà la funzione 'camera_callback'
        camera.listen(lambda image: camera_callback(image, image_data))
        print("Sensore fotocamera attivo. In attesa di immagini...")

        # --- 4. CICLO PRINCIPALE DI VISUALIZZAZIONE ---
        # Creiamo una finestra con OpenCV
        cv2.namedWindow('Camera Feed CARLA', cv2.WINDOW_AUTOSIZE)

        print("\nPremi 'q' sulla finestra della fotocamera per chiudere.")
        while True:
            # Avanza la simulazione. È FONDAMENTALE per generare nuovi dati dai sensori.
            world.tick()

            # Controlla se un'immagine è stata catturata dal callback
            if image_data['image'] is not None:
                cv2.imshow('Camera Feed CARLA', image_data['image'])

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
