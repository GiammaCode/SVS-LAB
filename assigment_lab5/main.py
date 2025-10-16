import carla
import time

import config
import sensor_callbacks
from carla_manager import CarlaManager
from spawner import Spawner

def main():
    #shared dictionary
    radar_data = {'min_ttc': float('inf')}

    with CarlaManager() as manager:
        spawner = Spawner(manager.world, manager.actor_list)

        #Spaning section
        ego_vehicle = spawner.spawn_vehicle(config.EGO_VEHICLE_MODEL)
        if not ego_vehicle: return

        # Spawn a target vehicle 10 meters in front of the ego vehicle
        carla_map = manager.world.get_map()
        ego_waypoint = carla_map.get_waypoint(ego_vehicle.get_location())

        # Get the next waypoint 10 meters down the road
        next_waypoint = ego_waypoint.next(20.0)[0]  # The [0] is important to get the first waypoint from the list

        # Add a small vertical offset to prevent spawning into the ground
        target_spawn_point = carla.Transform(
            next_waypoint.transform.location + carla.Location(z=0.1),
            next_waypoint.transform.rotation
        )

        target_vehicle = spawner.spawn_vehicle(config.TARGET_VEHICLE_MODEL, spawn_point=target_spawn_point)
        if not target_vehicle: return

        #Setting sensor and spectator
        spectator = manager.world.get_spectator()

        radar_sensor = spawner.spawn_radar(ego_vehicle)
        if not radar_sensor: return

        #start listen sensor
        radar_sensor.listen(lambda data: sensor_callbacks.radar_callback(data, radar_data))
        print("Radar sensor is activated")

        print("Start EBS test...")

        detection_counter = 0
        while True:
           #manager.world.tick()


            ego_transform = ego_vehicle.get_transform()
            spectator_location = ego_transform.transform(carla.Location(x=-8, z=3))
            spectator.set_transform(carla.Transform(spectator_location, ego_transform.rotation))

            current_ttc = radar_data['min_ttc']
            if current_ttc < config.TTC_THRESHOLD:
                detection_counter += 1
            else:
                detection_counter = 0

            if detection_counter >= config.STABLE_DETECTION_THRESHOLD:
                control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                ego_vehicle.apply_control(control)
                print(f"OBSTACLE DETECTED! TTC: {current_ttc:.2f}s! BRAKING ACTIVATED")
            else:
                control = carla.VehicleControl(throttle=1.0, brake=0.0, steer=0.0)
                ego_vehicle.apply_control(control)

                #if detection_counter > 0:
                #    print(f"Possible detection ({detection_counter}/{config.STABLE_DETECTION_THRESHOLD}) - TTC: {current_ttc:.2f}s")
                #else:
                #    print(f"No obstacle detected - TTC: {current_ttc:.2f}s")

            time.sleep(0.02)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nScript interrupted by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
