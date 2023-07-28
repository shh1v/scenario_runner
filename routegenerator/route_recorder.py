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
sys.path.append('../../carla/PythonAPI/examples/')
sys.path.append('../../carla/PythonAPI/experiment/')
import carla
from experiment_utils import ExperimentHelper
from DReyeVR_utils import find_ego_vehicle
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent

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
        DReyeVR_vehicle.set_transform(carla.Transform(carla.Location(8.5, 19.3, 0), carla.Rotation(0, -90.3, 0)))
        world.tick()
        
        new_transform = None
        old_transform = None
        with open("raw_waypoints/route_5.txt", "w") as file:
            while True:
                old_transform = new_transform
                new_transform = DReyeVR_vehicle.get_transform()
                if old_transform is not None and (is_nearly_equal(new_transform.location.x, old_transform.location.x, tolerance=0.01) and is_nearly_equal(new_transform.location.y, old_transform.location.y, tolerance=0.01) and is_nearly_equal(new_transform.location.z, old_transform.location.z, tolerance=0.01)):
                    # Don't write the same transform twice
                    world.tick()
                    continue
                file.write(str(new_transform) + "\n")
                file.flush()  # Force write to disk after each update.
                os.fsync(file.fileno())  # Ensure it's written to disk.
                world.tick()

    finally:
        pass


if __name__ == '__main__':
    try:
        main(host='127.0.0.1', port=2000, tm_port=8000)
    except KeyboardInterrupt:
        print('\ndone.')

