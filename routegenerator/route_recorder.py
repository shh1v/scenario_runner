#!/usr/bin/env python

# Copyright (c) 2023 Okanagan Visualization & Interaction (OVI) Lab
# The University of British Columbia, BC, Canada
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Script used to record the vehicle location into a txt file."""

# Standard library imports
import glob
import os
import time
import sys

# Local imports
import carla
from DReyeVR_utils import find_ego_vehicle

# Other library imports
import logging

def set_simulation_mode(client, synchronous_mode=True, tm_synchronous_mode=True, tm_port=8000, fixed_delta_seconds=0.05):
    # Setting simulation mode
    settings = client.get_world().get_settings()
    if synchronous_mode:
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = fixed_delta_seconds # 20 Hz
    else:
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
    client.get_world().apply_settings(settings)

    # Setting Traffic Manager parameters
    traffic_manager = client.get_trafficmanager(tm_port)
    traffic_manager.set_synchronous_mode(tm_synchronous_mode)

def main(**kargs):
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(kargs['host'], kargs['port'])
    client.set_timeout(10.0)
    world = client.load_world("Town05")
    
    try:
        # Setting actors starting position to the start of the route
        DReyeVR_vehicle = find_ego_vehicle(world)
        world.tick()
        
        new_transform = None
        last_logged_transform = None
        with open("routegenerator/route_data/raw_waypoints/study_route_1_town05.txt", "w") as file:
            while True:
                new_transform = DReyeVR_vehicle.get_transform()
                if last_logged_transform is not None and last_logged_transform.location.distance(new_transform.location) < 20:
                    # Don't write transforms that are too close together
                    world.tick()
                    continue
                world.debug.draw_string(new_transform.location, f"[000]\n[000]", draw_shadow=False,
                            color=carla.Color(r=255, g=0, b=0), life_time=10000.0,
                            persistent_lines=True)

                file.write(str(new_transform) + "\n")
                file.flush()  # Force write to disk after each update.
                os.fsync(file.fileno())  # Ensure it's written to disk.
                last_logged_transform = new_transform
                world.tick()
    finally:
        # Ensure world is again in stand alone
        set_simulation_mode(client, synchronous_mode=False, tm_synchronous_mode=False)


if __name__ == '__main__':
    try:
        main(host='127.0.0.1', port=2000, tm_port=8000)
    except KeyboardInterrupt:
        print('\ndone.')