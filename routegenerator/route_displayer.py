#!/usr/bin/env python

# Copyright (c) 2023 Okanagan Visualization & Interaction (OVI) Lab
# The University of British Columbia, BC, Canada
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Script used to display the route xml file in the simulation that was generated."""

# Standard library imports
import glob
import os
import time
import sys
sys.path.append('../carla/PythonAPI/examples/')
sys.path.append('../carla/PythonAPI/experiment/')

# Local imports
import carla
from experiment_utils import ExperimentHelper
from DReyeVR_utils import find_ego_vehicle
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent

# Other library imports
import xml.etree.ElementTree as ET
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
        # step when not recording driving performance or running any traffic scenarios

        # Setting actors starting position to the start of the route
        DReyeVR_vehicle = find_ego_vehicle(world)
        DReyeVR_vehicle.set_transform(carla.Transform(carla.Location(8.5, 19.3, 0), carla.Rotation(0, -90.3, 0)))
        world.tick()

        # assuming your xml is in a string, otherwise load it from a file
        xml_data = None
        with open('routegenerator/route_data/route_final_1.xml', 'r') as file:
            xml_data = file.read()

        root = ET.fromstring(xml_data)

        for route in root.findall('route'):
            route_id = route.get('id')
            town = route.get('town')
            print(f'Route ID: {route_id}, Town: {town}')
            
            for waypoint in route.findall('waypoint'):
                x = waypoint.get('x')
                y = waypoint.get('y')
                z = waypoint.get('z')
                pitch = waypoint.get('pitch')
                yaw = waypoint.get('yaw')
                roll = waypoint.get('roll')
                world.debug.draw_string(carla.Location(float(x), float(y), float(z)), 'O', draw_shadow=False,
                                            color=carla.Color(r=0, g=255, b=0), life_time=300.0,
                                            persistent_lines=True)

    finally:
        pass

if __name__ == '__main__':

    try:
        main(host='127.0.0.1', port=2000, tm_port=8000)
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
