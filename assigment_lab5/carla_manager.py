"Manages the connection to the CARLA server and actor cleanup"

import carla
import config

class CarlaManager:
    def __init__(self):
        self.client = None
        self.world = None
        self.actor_list = []

    def __enter__(self):
        """Connection to CARLA server and gets the world"""
        print("Connecting to CARLA...")
        self.client = carla.Client(config.HOST, config.PORT)
        self.client.set_timeout(config.TIMEOUT)
        self.world = self.client.get_world()
        print("Connection successfully")
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Destroys all tracked actors to clean up the simulation"""
        print("\nCleaning up actors...")
        for actor in self.actor_list:
            if actor.is_alive:
                print(f"Destroying actor {actor.type_id}")
                actor.destroy()
        print("cleanup completed")
