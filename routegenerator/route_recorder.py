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
sys.path.append('../carla/PythonAPI/examples/')
sys.path.append('../carla/PythonAPI/experiment/')
import carla
from experiment_utils import ExperimentHelper
from DReyeVR_utils import find_ego_vehicle

# Other library imports
import logging

def main(**kargs):
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(kargs['host'], kargs['port'])
    client.set_timeout(10.0)
    world = client.get_world()
    
    try:
        # Simulation Synchronization
        ExperimentHelper.set_simulation_mode(client, synchronous_mode=True, tm_synchronous_mode=True, tm_port=8000, fixed_delta_seconds=0.05)

        # Setting actors starting position to the start of the route
        DReyeVR_vehicle = find_ego_vehicle(world)
        world.tick()
        
        new_transform = None
        last_logged_transform = None
        with open("routegenerator/raw_waypoints/study_route_2.txt", "w") as file:
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
        ExperimentHelper.set_simulation_mode(client, synchronous_mode=False, tm_synchronous_mode=False)


if __name__ == '__main__':
    try:
        main(host='127.0.0.1', port=2000, tm_port=8000)
    except KeyboardInterrupt:
        print('\ndone.')

