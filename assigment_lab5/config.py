"""
Configuration constants for the CARLA simulation.
"""

#Server connection
HOST = 'localhost'
PORT = 2000
TIMEOUT = 10

EGO_VEHICLE_MODEL = 'vehicle.audi.tt'
TARGET_VEHICLE_MODEL = 'vehicle.volkswagen.t2'

# Emergency Braking System (EBS) settings
TTC_THRESHOLD = 2 #seconds
STABLE_DETECTION_THRESHOLD = 3 #number of consecutive "true"

#RADAR value
RADAR_RANGE = 50.0 #meters
RADAR_HORIZONTAL_FOV = 45 #degrees
RADAR_VERTICAL_FOV = 30 #degrees
RADAR_POINTS_PER_SECOND = 1500



