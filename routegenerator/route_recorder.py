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
import argparse
import logging

def is_nearly_equal(a, b, tolerance=0.01):
    return abs(a - b) < tolerance
def main(**kargs):
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(kargs['host'], kargs['port'])
    client.set_timeout(10.0)
    synchronous_master = False

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(kargs['tm_port'])
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_respawn_dormant_vehicles(True) # See
        traffic_manager.set_hybrid_physics_mode(True) # TODO: Set DReyeVR's role to hero
        traffic_manager.set_hybrid_physics_radius(70.0)

        # Simulation Synchronization
        ExperimentHelper.set_synchronous_mode(world)

        # Setting actors starting position to the start of the route
        DReyeVR_vehicle = find_ego_vehicle(world)
        world.tick()
        
        new_transform = None
        last_logged_transform = None
        with open("routegenerator/raw_waypoints/route_final_2.txt", "w") as file:
            while True:
                new_transform = DReyeVR_vehicle.get_transform()
                if  last_logged_transform is not None and last_logged_transform.location.distance(new_transform.location) < 20:
                    # Don't write transforms that are too close together
                    world.tick()
                    continue
                file.write(str(new_transform) + "\n")
                file.flush()  # Force write to disk after each update.
                os.fsync(file.fileno())  # Ensure it's written to disk.
                last_logged_transform = new_transform
                world.tick()

    finally:
        pass


if __name__ == '__main__':
    try:
        main(host='127.0.0.1', port=2000, tm_port=8000)
    except KeyboardInterrupt:
        print('\ndone.')

