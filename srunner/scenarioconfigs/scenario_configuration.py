#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the key configuration parameters for an XML-based scenario
"""

import carla


class ActorConfigurationData(object):

    """
    This is a configuration base class to hold model and transform attributes
    """

    def __init__(self, model, transform, rolename='other', init_speed=0, final_speed=0, speed=0, autopilot=False,
                 random=False, color=None, category="car", args=None, lane=None, role=None, vehicle_offset=None):
        self.model = model
        self.rolename = rolename
        self.transform = transform
        self.init_speed = init_speed
        self.final_speed = final_speed
        self.speed = speed
        self.autopilot = autopilot
        self.random_location = random
        self.color = color
        self.category = category
        self.args = args

        # Custom attributes for AutoHive Implementation
        self.lane = lane
        self.role = role
        self.vehicle_offset = vehicle_offset

    @staticmethod
    def parse_from_node(node, rolename):
        """
        static method to initialize an ActorConfigurationData from a given ET tree
        """

        model = node.attrib.get('model', '*vehicle.*')

        pos_x = float(node.attrib.get('x', 0))
        pos_y = float(node.attrib.get('y', 0))
        pos_z = float(node.attrib.get('z', 0))
        yaw = float(node.attrib.get('yaw', 0))

        transform = carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z), carla.Rotation(yaw=yaw))

        rolename = node.attrib.get('rolename', rolename)


        speed = float(node.attrib.get('speed', 0))
        init_speed = float(node.attrib.get('init_speed', 0))
        final_speed = float(node.attrib.get('final_speed', 0))

        autopilot = False
        if 'autopilot' in node.keys():
            autopilot = True

        random_location = False
        if 'random_location' in node.keys():
            random_location = True

        color = node.attrib.get('color', None)

        # Custom attrbutes for AutoHive Implementation
        lane = node.attrib.get('lane', None)
        role = node.attrib.get('role', None)
        vehicle_offset = float(node.attrib.get('vehicle_offset', None))

        if (lane != None or role != None or vehicle_offset != None or speed != None):
            # This is a vehicle for a take-over scenario
            return ActorConfigurationData(model=model, transform=transform, init_speed=init_speed, final_speed=final_speed, lane=lane, role=role, vehicle_offset=vehicle_offset)
        else:
            return ActorConfigurationData(model=model, transform=transform, rolename=rolename, speed=speed, autopilot=autopilot, random=random_location, color=color)


class ScenarioConfiguration(object):

    """
    This class provides a basic scenario configuration incl.:
    - configurations for all actors
    - town, where the scenario should be executed
    - name of the scenario (e.g. ControlLoss_1)
    - type is the class of scenario (e.g. ControlLoss)
    """

    trigger_points = []
    ego_vehicles = []
    other_actors = []
    town = None
    name = None
    type = None
    route = None
    agent = None
    weather = carla.WeatherParameters()
    friction = None
    subtype = None
    route_var_name = None

    # Custom AutoHive attributes
    scenario_manager = None
    traffic_manager = None
