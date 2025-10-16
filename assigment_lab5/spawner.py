"""Spawner handles the generation of actors """

import carla
import random
import config


class Spawner:
    "handles the generation of actors"
    def __init__(self, world, actor_list):
        self.world = world
        self.actor_list = actor_list
        self.blueprint_library = self.world.get_blueprint_library()


    def spawn_vehicle(self, model, spawn_point=None, autopilot=False):
        """Spawn veichles according to the model at a specific random point

        Returns:
            Carla.Actor: spawned veichles actor, or None on failure.
        """
        vehicle_bp = self.blueprint_library.filter(model)[0]

        if spawn_point is None:
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                print("No spawn points, error")
                return None
            spawn_point = random.choice(spawn_points)

        vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)

        if vehicle:
            print(f"Spawn succeeded, model: {model}")
            vehicle.set_autopilot(autopilot)
            self.actor_list.append(vehicle)
        else:
            print(f"ERROR: Spawn failed, model: {model}")
        return vehicle


    def spawn_radar(self, parent_vehicle):
        """Spawn radar according to the model at a specific random point"""
        radar_bp = self.blueprint_library.find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', str(config.RADAR_HORIZONTAL_FOV))
        radar_bp.set_attribute('vertical_fov', str(config.RADAR_VERTICAL_FOV))
        radar_bp.set_attribute('points_per_second', str(config.RADAR_POINTS_PER_SECOND))
        radar_bp.set_attribute('range', str(config.RADAR_RANGE))

        radar_transform = carla.Transform(carla.Location(x=2.5, z=1.0))
        radar_sensor = self.world.spawn_actor(
            radar_bp,
            radar_transform,
            attach_to=parent_vehicle
        )

        if radar_sensor:
            print("Radar sensor spawned and attached to vehicle")
            self.actor_list.append(radar_sensor)
        else:
            print("ERROR: Radar sensor spawn failed")

        return radar_sensor