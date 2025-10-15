"""Callback function for sprocessing data from CARLA sensor"""

import numpy as np
import math

def radar_callback(radar_data, data_dict):
    """
    Callback function for the radar sensor
    Processes radar detections to find the minimum Time To Collision
    """

    min_ttc = float('inf')
    # process each detection from the radar sweep
    for detection in radar_data:
        distance = detection.depth
        relative_velocity  = detection.velocity

        # We are interested in objects we are approaching.
        # Radar velocity is positive for objects moving away, negative for objects approaching.
        # We define closing_speed as positive when approaching.
        closing_speed = -relative_velocity

        if closing_speed > 0.1:
            ttc  = distance / closing_speed
            min_ttc = min(min_ttc, ttc)

    data_dict['min_ttc'] = min_ttc
