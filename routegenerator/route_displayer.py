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
    
    # Change this to the route xml file you want to display
    file_name = 'routegenerator/route_data/take_over_routes_short.xml'

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(kargs['tm_port'])
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_respawn_dormant_vehicles(True) # See
        traffic_manager.set_hybrid_physics_mode(True) # TODO: Set DReyeVR's role to hero
        traffic_manager.set_hybrid_physics_radius(70.0)

        client.reload_world()

        # Simulation Syncronization
        #
        # Run the simulation in asynchronous mode with variable time (default)
        # step when not recording driving performance or running any traffic scenarios.
        #
        # Run the simulation in synchronous mode with fixed time
        # step when not recording driving performance or running any traffic scenarios

        # Setting actors starting position to the start of the route
        DReyeVR_vehicle = find_ego_vehicle(world)

        # assuming your xml is in a string, otherwise load it from a file
        xml_data = None
        with open(file_name, 'r') as file:
            xml_data = file.read()

        root = ET.fromstring(xml_data)
        isFirst = True
        for route in root.findall('route'):
            route_id = route.get('id')
            town = route.get('town')
            print(f'Route ID: {route_id}, Town: {town}')
            
            for waypoint in route.findall('waypoint'):
                if isFirst:
                    DReyeVR_vehicle.set_transform(carla.Transform(carla.Location(float(waypoint.get('x')), float(waypoint.get('y')), float(waypoint.get('z'))), carla.Rotation(float(waypoint.get('pitch')), float(waypoint.get('yaw')), float(waypoint.get('roll')))))
                    isFirst = False
                x = waypoint.get('x')
                y = waypoint.get('y')
                z = waypoint.get('z')
                # world.debug.draw_string(carla_waypoint.transform.location, f"[0]", draw_shadow=False,
                #                             color=carla.Color(r=0, g=255, b=0), life_time=300.0,
                #                             persistent_lines=True)
                ################################### OR #########################################
                carla_waypoint = world.get_map().get_waypoint(carla.Location(float(x), float(y), float(z)))
                world.debug.draw_string(carla_waypoint.transform.location, f"[*] {carla_waypoint.transform.location}\n{carla_waypoint.transform.rotation}", draw_shadow=False,
                            color=carla.Color(r=255, g=0, b=0), life_time=300.0,
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
