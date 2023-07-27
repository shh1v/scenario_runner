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
sys.path.append('../../carla/PythonAPI/examples/')
sys.path.append('../../carla/PythonAPI/experiment/')

# Local imports
import carla
from experiment_utils import ExperimentHelper
from DReyeVR_utils import find_ego_vehicle
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent

# Other library imports
import argparse
import logging

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

        # Simulation Syncronization
        #
        # Run the simulation in asynchronous mode with variable time (default)
        # step when not recording driving performance or running any traffic scenarios.
        #
        # Run the simulation in synchronous mode with fixed time
        # step when not recording driving performance or running any traffic scenarios.
        ExperimentHelper.set_synchronous_mode(world)

        # waypoints = world.get_map().generate_waypoints(1)
        # for w in waypoints:
        #     world.debug.draw_string(w.transform.location, 'O', draw_shadow=False,
        #                                     color=carla.Color(r=255, g=0, b=0), life_time=120.0,
        #                                     persistent_lines=True)

        # Setting actors starting position to the start of the route
        DReyeVR_vehicle = find_ego_vehicle(world)
        DReyeVR_vehicle.set_transform(carla.Transform(carla.Location(8.5, 19.3, 0), carla.Rotation(0, -90.3, 0)))
        world.tick()

        with open("raw_waypoints/route_4.txt", "a") as file:
            new_transform = None
            old_transform = None
            while True:
                old_transform = new_transform
                new_transform = DReyeVR_vehicle.get_transform()
                if old_transform is not None and (new_transform.location == old_transform.location):
                    # Dont write the same transform twice
                    world.tick()
                    continue
                elif old_transform is not None and (old_transform.location.distance(new_transform.location) < 0.5):
                    # Dont write transform that are too close together
                    world.tick()
                    continue
                try:
                    # Open the file in append mode
                    file.write(str(new_transform))
                    file.write("\n")
                except Exception as e:
                    print("An error occurred:", str(e))
                    break
                world.tick()
    finally:
        pass

if __name__ == '__main__':

    try:
        main(host='127.0.0.1', port=2000, tm_port=8000)
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
