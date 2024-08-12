#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides Challenge routes as standalone scenarios
"""

from __future__ import print_function

import math
import json
import os
import traceback
import xml.etree.ElementTree as ET

import py_trees

import carla

from agents.navigation.local_planner import RoadOption

# DReyeVR import
from examples.DReyeVR_utils import find_ego_vehicle

# pylint: disable=line-too-long
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration, ActorConfigurationData
# pylint: enable=line-too-long
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle, ScenarioTriggerer, SendVehicleStatus, ChangeVehicleStatus, SpawnNPCsAtTransform
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.route_parser import RouteParser, TRIGGER_THRESHOLD, TRIGGER_ANGLE_THRESHOLD
from srunner.tools.route_manipulation import interpolate_trajectory
from srunner.tools.py_trees_port import oneshot_behavior

from srunner.scenarios.control_loss import ControlLoss
from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicleRoute
from srunner.scenarios.object_crash_vehicle import DynamicObjectCrossing
from srunner.scenarios.object_crash_intersection import VehicleTurningRoute
from srunner.scenarios.other_leading_vehicle import OtherLeadingVehicle
from srunner.scenarios.maneuver_opposite_direction import ManeuverOppositeDirection
from srunner.scenarios.junction_crossing_route import NoSignalJunctionCrossingRoute
from srunner.scenarios.signalized_junction_left_turn import SignalizedJunctionLeftTurn
from srunner.scenarios.signalized_junction_right_turn import SignalizedJunctionRightTurn
from srunner.scenarios.opposite_vehicle_taking_priority import OppositeVehicleRunningRedLight
from srunner.scenarios.traffic_complexity import TrafficComplexity
from srunner.scenarios.lane_change_tor import LaneChangeTOR
from srunner.scenarios.background_activity import BackgroundActivity

from srunner.scenariomanager.scenarioatomics.atomic_criteria import (CollisionTest,
                                                                     InRouteTest,
                                                                     RouteCompletionTest,
                                                                     OutsideRouteLanesTest,
                                                                     RunningRedLightTest,
                                                                     RunningStopTest,
                                                                     ActorSpeedAboveThresholdTest)

SECONDS_GIVEN_PER_METERS = 0.4

NUMBER_CLASS_TRANSLATION = {
    "Scenario1": ControlLoss,
    "Scenario2": FollowLeadingVehicleRoute,
    "Scenario3": DynamicObjectCrossing,
    "Scenario4": VehicleTurningRoute,
    "Scenario5": OtherLeadingVehicle,
    "Scenario6": ManeuverOppositeDirection,
    "Scenario7": OppositeVehicleRunningRedLight,
    "Scenario8": SignalizedJunctionLeftTurn,
    "Scenario9": SignalizedJunctionRightTurn,
    "Scenario10": NoSignalJunctionCrossingRoute,
    "Scenario11": TrafficComplexity,
    "Scenario12": LaneChangeTOR
}


def convert_json_to_transform(actor_dict):
    """
    Convert a JSON string to a CARLA transform
    """
    return carla.Transform(location=carla.Location(x=float(actor_dict['x']), y=float(actor_dict['y']),
                                                   z=float(actor_dict['z'])),
                           rotation=carla.Rotation(roll=0.0, pitch=0.0, yaw=float(actor_dict['yaw'])))


def convert_json_to_actor(actor_dict):
    """
    Convert a JSON string to an ActorConfigurationData dictionary
    """
    node = ET.Element('waypoint')
    
    # AutoHive: Handled the case where the following attributes are not provided
    # NOTE: Instead of pass, we can also print a warning message
    try:
        node.set('x', actor_dict['x'])
    except KeyError:
        pass
    try:
        node.set('y', actor_dict['y'])
    except KeyError:
        pass
    try:
        node.set('z', actor_dict['z'])
    except KeyError:
        pass
    try:
        node.set('yaw', actor_dict['yaw'])
    except KeyError:
        pass

    # AutoHive: Loading custom attributes
    try:
        node.set('model', actor_dict['model'])
    except KeyError:
        pass
    try:
        node.set('lane', actor_dict['lane'])
    except KeyError:
        pass
    try:
        node.set('role', actor_dict['role'])
    except KeyError:
        pass
    try:
        node.set('vehicle_offset', actor_dict['vehicle_offset'])
    except KeyError:
        pass
    try:
        node.set('init_speed', actor_dict['init_speed'])
    except KeyError:
        pass
    try:
        node.set('final_speed', actor_dict['final_speed'])
    except KeyError:
        pass


    return ActorConfigurationData.parse_from_node(node, 'simulation')


def convert_transform_to_location(transform_vec):
    """
    Convert a vector of transforms to a vector of locations
    """
    location_vec = []
    for transform_tuple in transform_vec:
        location_vec.append((transform_tuple[0].location, transform_tuple[1]))

    return location_vec


def compare_scenarios(scenario_choice, existent_scenario):
    """
    Compare function for scenarios based on distance of the scenario start position
    """
    def transform_to_pos_vec(scenario):
        """
        Convert left/right/front to a meaningful CARLA position
        """
        position_vec = [scenario['trigger_position']]
        if scenario['other_actors'] is not None:
            if 'left' in scenario['other_actors']:
                position_vec += scenario['other_actors']['left']
            if 'front' in scenario['other_actors']:
                position_vec += scenario['other_actors']['front']
            if 'right' in scenario['other_actors']:
                position_vec += scenario['other_actors']['right']

        return position_vec

    # put the positions of the scenario choice into a vec of positions to be able to compare

    choice_vec = transform_to_pos_vec(scenario_choice)
    existent_vec = transform_to_pos_vec(existent_scenario)
    for pos_choice in choice_vec:
        for pos_existent in existent_vec:

            dx = float(pos_choice['x']) - float(pos_existent['x'])
            dy = float(pos_choice['y']) - float(pos_existent['y'])
            dz = float(pos_choice['z']) - float(pos_existent['z'])
            dist_position = math.sqrt(dx * dx + dy * dy + dz * dz)
            dyaw = float(pos_choice['yaw']) - float(pos_choice['yaw'])
            dist_angle = math.sqrt(dyaw * dyaw)
            if dist_position < TRIGGER_THRESHOLD and dist_angle < TRIGGER_ANGLE_THRESHOLD:
                return True

    return False


class RouteScenario(BasicScenario):

    """
    Implementation of a RouteScenario, i.e. a scenario that consists of driving along a pre-defined route,
    along which several smaller scenarios are triggered
    """

    def __init__(self, world, config, debug_mode=False, criteria_enable=True, timeout=300, background_activity=False, scenario_manager=None, traffic_manager=None):
        """
        Setup all relevant parameters and create scenarios along route
        """

        self.config = config
        self.route = None
        self.sampled_scenarios_definitions = None

        self._update_route(world, config, debug_mode)

        self.ego_vehicle = self._initialize_ego_vehicle_dreyevr(find_ego_vehicle(world))

        # Custom AutoHive Implementation: add scenario manager
        self.scenario_manager = scenario_manager
        self.traffic_manager = traffic_manager

        self.list_scenarios = self._build_scenario_instances(world,
                                                             self.ego_vehicle,
                                                             self.sampled_scenarios_definitions,
                                                             scenarios_per_tick=5,
                                                             timeout=self.timeout,
                                                             debug_mode=debug_mode)

        if background_activity:
            self.list_scenarios.append(BackgroundActivity(
                world, self.ego_vehicle, self.config, self.route, timeout=self.timeout))

        super(RouteScenario, self).__init__(name=config.name,
                                            ego_vehicles=[self.ego_vehicle],
                                            config=config,
                                            world=world,
                                            debug_mode=False,
                                            terminate_on_failure=False,
                                            criteria_enable=criteria_enable)

    def _update_route(self, world, config, debug_mode):
        """
        Update the input route, i.e. refine waypoint list, and extract possible scenario locations

        Parameters:
        - world: CARLA world
        - config: Scenario configuration (RouteConfiguration)
        """

        # Transform the scenario file into a dictionary
        world_annotations = RouteParser.parse_annotations_file(config.scenario_file)

        # prepare route's trajectory (interpolate and add the GPS route)
        gps_route, route = interpolate_trajectory(config.trajectory)

        potential_scenarios_definitions, _ = RouteParser.scan_route_for_scenarios(config.town, route, world_annotations)

        self.route = route
        CarlaDataProvider.set_ego_vehicle_route(convert_transform_to_location(self.route))

        if config.agent is not None:
            config.agent.set_global_plan(gps_route, self.route)

        # Sample the scenarios to be used for this route instance.
        self.sampled_scenarios_definitions = self._scenario_sampling(potential_scenarios_definitions)

        # Timeout of scenario in seconds
        self.timeout = self._estimate_route_timeout()

        # Print route in debug mode
        if debug_mode:
            self._draw_waypoints(world, self.route, vertical_shift=0.1, persistency=50000.0)

        self._setup_nav_signs(self.route)

    def _initialize_ego_vehicle_dreyevr(self, ego_vehicle):
        """
        Set/Update the start position of the ego_vehicle (instead of _update_ego_vehicle below)
        """
        # move ego to correct position
        elevate_transform = self.route[0][0]
        elevate_transform.location.z += 0.5
        ego_vehicle.set_transform(elevate_transform)
        CarlaDataProvider.register_actor(ego_vehicle)
        return ego_vehicle

    def _update_ego_vehicle(self):
        """
        Set/Update the start position of the ego_vehicle
        """
        # move ego to correct position
        elevate_transform = self.route[0][0]
        elevate_transform.location.z += 0.5

        ego_vehicle = CarlaDataProvider.request_new_actor('vehicle.lincoln.mkz2017',
                                                          elevate_transform,
                                                          rolename='hero')

        return ego_vehicle

    def _estimate_route_timeout(self):
        """
        Estimate the duration of the route
        """
        route_length = 0.0  # in meters

        prev_point = self.route[0][0]
        for current_point, _ in self.route[1:]:
            dist = current_point.location.distance(prev_point.location)
            route_length += dist
            prev_point = current_point

        return int(SECONDS_GIVEN_PER_METERS * route_length)

    # pylint: disable=no-self-use
    def _draw_waypoints(self, world, waypoints, vertical_shift, persistency=-1):
        """
        Draw a list of waypoints at a certain height given in vertical_shift.
        """
        for w in waypoints:
            wp = w[0].location + carla.Location(z=vertical_shift)

            if w[1] == RoadOption.LEFT:  # Yellow
                color = carla.Color(255, 255, 0)
            elif w[1] == RoadOption.RIGHT:  # Cyan
                color = carla.Color(0, 255, 255)
            elif w[1] == RoadOption.CHANGELANELEFT:  # Orange
                color = carla.Color(255, 64, 0)
            elif w[1] == RoadOption.CHANGELANERIGHT:  # Dark Cyan
                color = carla.Color(0, 64, 255)
            elif w[1] == RoadOption.STRAIGHT:  # Gray
                color = carla.Color(128, 128, 128)
            else:  # LANEFOLLOW
                color = carla.Color(0, 255, 0)  # Green

            world.debug.draw_point(wp, size=0.1, color=color, life_time=persistency)

        world.debug.draw_point(waypoints[0][0].location + carla.Location(z=vertical_shift), size=0.2,
                               color=carla.Color(0, 0, 255), life_time=persistency)
        world.debug.draw_point(waypoints[-1][0].location + carla.Location(z=vertical_shift), size=0.2,
                               color=carla.Color(255, 0, 0), life_time=persistency)

    def _get_valid_sign_transform(self, wp_transform):
        # use the original waypoint location as long as it is on a sidewalk
        _wp = CarlaDataProvider.get_map().get_waypoint(wp_transform.location,
                                                       project_to_road=False,
                                                       lane_type=carla.LaneType.Any)
        if _wp is None:
            return None
        # find the first non-road waypoint so our drivers can read it (with a limit)
        max_tries: int = 100
        valid_sign_lanes = [
            carla.LaneType.Sidewalk,
            # carla.LaneType.Shoulder, # disable shoulder
        ]
        while max_tries > 0 and _wp.lane_type not in valid_sign_lanes:
            max_tries -= 1
            right_wp = _wp.get_right_lane()
            if right_wp is not None:
                _wp = right_wp
            else:
                continue  # skip this one
        # carla transforms don't have a native assignment operator :/
        t = carla.Transform(_wp.transform.location, _wp.transform.rotation)
        if max_tries == 0:  # didn't find a sidewalk, so push it slightly right by the road lane width
            push = t.get_right_vector() * _wp.lane_width * 1.1  # 10% more just to be safe
            # carla locations don't have a native += operator
            t.location = carla.Location(x=t.location.x + push.x,
                                        y=t.location.y + push.y,
                                        z=t.location.z + push.z)
        t.location.z += 2.0  # go up slightly (for the height of the road sign)
        t.rotation.yaw += 90.0  # rotate another 90 degrees
        return t

    def _setup_nav_signs(self, waypoints: list):
        """
        Draw the signs along the waypoints of a route automatically
        """
        prev_sign_type = None  # only request new actor if nav type changes
        for i, w in enumerate(waypoints):
            wp_transform, sign_type = w
            if prev_sign_type is not None and prev_sign_type == sign_type:
                continue  # only spawn a sign when the waypoint type changes
            prev_sign_type = sign_type
            sign_transform = self._get_valid_sign_transform(wp_transform)
            if sign_transform is None:
                continue  # invalid
            # now we can finally go about spawning the sign in this location
            dreyevr_sign_type: str = "dreyevr_sign_straight"
            if sign_type == RoadOption.LEFT:
                dreyevr_sign_type = "dreyevr_sign_left"
            elif sign_type == RoadOption.RIGHT:
                dreyevr_sign_type = "dreyevr_sign_right"
            elif sign_type == RoadOption.CHANGELANELEFT:
                continue
            elif sign_type == RoadOption.CHANGELANERIGHT:
                continue
            elif sign_type == RoadOption.STRAIGHT:
                dreyevr_sign_type = "dreyevr_sign_straight"
            else:
                continue
            sign_type: str = f"static.prop.{dreyevr_sign_type}"
            # Commented out to avoid log spam
            # print(f"Spawning ({sign_type}) sign {i} at {sign_transform}")
            CarlaDataProvider.request_new_actor(sign_type, sign_transform,
                                                rolename='navigation_sign')
        # plot the final goal waypoint (at the end)
        wp_transform_final = waypoints[-1][0]
        goal_sign_transform = self._get_valid_sign_transform(wp_transform_final)
        if goal_sign_transform is not None:
            sign_type: str = "static.prop.dreyevr_sign_goal"
            print(f"Spawning ({sign_type}) sign {len(waypoints) - 1} at {goal_sign_transform}")
            CarlaDataProvider.request_new_actor(sign_type,
                                                goal_sign_transform,
                                                rolename='navigation_sign')

    def _scenario_sampling(self, potential_scenarios_definitions, random_seed=0):
        """
        The function used to sample the scenarios that are going to happen for this route.
        """

        # fix the random seed for reproducibility
        rng = CarlaDataProvider.get_random_seed()

        def position_sampled(scenario_choice, sampled_scenarios):
            """
            Check if a position was already sampled, i.e. used for another scenario
            """
            for existent_scenario in sampled_scenarios:
                # If the scenarios have equal positions then it is true.
                if compare_scenarios(scenario_choice, existent_scenario):
                    return True

            return False

        # The idea is to randomly sample a scenario per trigger position.
        sampled_scenarios = []
        for trigger in potential_scenarios_definitions.keys():
            possible_scenarios = potential_scenarios_definitions[trigger]

            scenario_choice = rng.choice(possible_scenarios)
            del possible_scenarios[possible_scenarios.index(scenario_choice)]
            # We keep sampling and testing if this position is present on any of the scenarios.
            while position_sampled(scenario_choice, sampled_scenarios):
                if possible_scenarios is None or not possible_scenarios:
                    scenario_choice = None
                    break
                scenario_choice = rng.choice(possible_scenarios)
                del possible_scenarios[possible_scenarios.index(scenario_choice)]

            if scenario_choice is not None:
                sampled_scenarios.append(scenario_choice)

        return sampled_scenarios

    def _build_scenario_instances(self, world, ego_vehicle, scenario_definitions,
                                  scenarios_per_tick=5, timeout=300, debug_mode=False):
        """
        Based on the parsed route and possible scenarios, build all the scenario classes.
        """
        scenario_instance_vec = []

        if debug_mode:
            for scenario in scenario_definitions:
                loc = carla.Location(scenario['trigger_position']['x'],
                                     scenario['trigger_position']['y'],
                                     scenario['trigger_position']['z']) + carla.Location(z=2.0)
                world.debug.draw_point(loc, size=0.3, color=carla.Color(255, 0, 0), life_time=100000)
                world.debug.draw_string(loc, str(scenario['name']), draw_shadow=False,
                                        color=carla.Color(0, 0, 255), life_time=100000, persistent_lines=True)

        for scenario_number, definition in enumerate(scenario_definitions):
            # Get the class possibilities for this scenario number
            scenario_class = NUMBER_CLASS_TRANSLATION[definition['name']]

            # Create the other actors that are going to appear
            if definition['other_actors'] is not None:
                list_of_actor_conf_instances = self._get_actors_instances(definition['other_actors'])
            else:
                list_of_actor_conf_instances = []
            # Create an actor configuration for the ego-vehicle trigger position

            egoactor_trigger_position = convert_json_to_transform(definition['trigger_position'])
            scenario_configuration = ScenarioConfiguration()
            scenario_configuration.other_actors = list_of_actor_conf_instances
            scenario_configuration.trigger_points = [egoactor_trigger_position]
            scenario_configuration.subtype = definition['scenario_type']
            scenario_configuration.ego_vehicles = [ActorConfigurationData(ego_vehicle.type_id,
                                                                          ego_vehicle.get_transform(),
                                                                          'hero')]
            route_var_name = "ScenarioRouteNumber{}".format(scenario_number)
            scenario_configuration.route_var_name = route_var_name
            # Custom AutoHive Implementation: add scenario manager
            scenario_configuration.scenario_manager = self.scenario_manager
            scenario_configuration.traffic_manager = self.traffic_manager
            try:
                scenario_instance = scenario_class(world=world, ego_vehicles=[ego_vehicle], config=scenario_configuration,
                                                   criteria_enable=False, timeout=timeout)
                # Do a tick every once in a while to avoid spawning everything at the same time
                if scenario_number % scenarios_per_tick == 0:
                    if CarlaDataProvider.is_sync_mode():
                        world.tick()
                    else:
                        world.wait_for_tick()

                scenario_number += 1
            except Exception as e:      # pylint: disable=broad-except
                if debug_mode:
                    traceback.print_exc()
                print("Skipping scenario '{}' due to setup error: {}".format(definition['name'], e))
                continue

            scenario_instance_vec.append(scenario_instance)

        return scenario_instance_vec

    def _get_actors_instances(self, list_of_antagonist_actors):
        """
        Get the full list of actor instances.
        """

        def get_actors_from_list(list_of_actor_def):
            """
                Receives a list of actor definitions and creates an actual list of ActorConfigurationObjects
            """
            sublist_of_actors = []
            for actor_def in list_of_actor_def:
                sublist_of_actors.append(convert_json_to_actor(actor_def))

            return sublist_of_actors

        # Custom AutoHive Implementation: parse all the vehicles
        list_of_actors = get_actors_from_list(list_of_antagonist_actors)

        return list_of_actors

    # pylint: enable=no-self-use

    def _initialize_actors(self, config):
        """
        Set other_actors to the superset of all scenario actors
        """
        # Create the background activity of the route
        town_amount = {
            'Town01': 0,
            'Town02': 0,
            'Town03': 0,
            'Town04': 0,
            'Town05': 0,
            'Town06': 0,
            'Town07': 0,
            'Town08': 0,
            'Town09': 0,
            'Town10': 0,
        }

        amount = town_amount[config.town] if config.town in town_amount else 0

        new_actors = CarlaDataProvider.request_new_batch_actors('vehicle.*',
                                                                amount,
                                                                carla.Transform(),
                                                                autopilot=True,
                                                                random_location=True,
                                                                rolename='background')

        if new_actors is None:
            raise RuntimeError("Error: Unable to add the background activity, all spawn points were occupied")
        else:
            print(f"Added {len(new_actors)} background actors")

        if not hasattr(self, "other_actors"):
            self.other_actors = []

        for _actor in new_actors:
            self.other_actors.append(_actor)

        # Add all the actors of the specific scenarios to self.other_actors
        for scenario in self.list_scenarios:
            self.other_actors.extend(scenario.other_actors)

    def _create_behavior(self):
        """
        Basic behavior do nothing, i.e. Idle
        """
        scenario_trigger_distance = 1.5  # Max trigger distance between route and scenario

        behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        subbehavior = py_trees.composites.Parallel(name="Behavior",
                                                   policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        scenario_behaviors = []
        blackboard_list = []

        for i, scenario in enumerate(self.list_scenarios):
            if scenario.scenario.behavior is not None:
                route_var_name = scenario.config.route_var_name
                if route_var_name is not None:
                    scenario_behaviors.append(scenario.scenario.behavior)
                    blackboard_list.append([scenario.config.route_var_name,
                                            scenario.config.trigger_points[0].location])
                else:
                    name = "{} - {}".format(i, scenario.scenario.behavior.name)
                    oneshot_idiom = oneshot_behavior(name,
                                                     behaviour=scenario.scenario.behavior,
                                                     name=name)
                    scenario_behaviors.append(oneshot_idiom)

        # Add behavior that manages the scenarios trigger conditions
        scenario_triggerer = ScenarioTriggerer(
            self.ego_vehicles[0],
            self.route,
            blackboard_list,
            scenario_trigger_distance,
            repeat_scenarios=False
        )

        subbehavior.add_child(scenario_triggerer)  # make ScenarioTriggerer the first thing to be checked
        subbehavior.add_children(scenario_behaviors)
        subbehavior.add_child(Idle())  # The behaviours cannot make the route scenario stop

        # Add behaviour to send the vehicle status in parallel to the scenario behaviour
        send_vehicle_status = SendVehicleStatus()
        subbehavior.add_child(send_vehicle_status)

        # Add behaviour to send signal for ego vehicle tunring on autopilot
        change_to_autopilot_status = ChangeVehicleStatus(vehicle_status="Autopilot")
        subbehavior.add_child(change_to_autopilot_status)
        
        behavior.add_child(subbehavior)

        # Spawn the npc vehicles for realistic effect and enable waypoint follower
        # Note: the following atomic behaviours are added to the behaviour node instead of the subbehaviour
        # since behaviour is uses SUCCESS_ON_ONE policy.

        npc_behaviour = 

        for actor_config in self.config.npc_actors:
            
            npc_behaviour = py_trees.composites.Sequence(f"Vehicle Parameters Setter: for npc vehicle id: {actor_config.model}")

            # Spawn the npc vehicle
            spawn_vehicle = SpawnNPCsAtTransform(relative_vehicle=self.config.ego_spawn_point, actor_config=actor_config)
            npc_behaviour.add_child(spawn_vehicle)

            # Enable waypoint follower (to have autopilot effect)

            # Lastly, add to the behaviour node
            behavior.add_child(spawn_vehicle)
        return behavior

    def _create_test_criteria(self):
        """
        """

        criteria = []

        route = convert_transform_to_location(self.route)

        collision_criterion = CollisionTest(self.ego_vehicles[0], terminate_on_failure=False)

        route_criterion = InRouteTest(self.ego_vehicles[0],
                                      route=route,
                                      offroad_max=100,
                                      terminate_on_failure=False)

        completion_criterion = RouteCompletionTest(self.ego_vehicles[0], route=route)

        # Commeting out unused criterias to get avoid unnecessary return of failure.

        # outsidelane_criterion = OutsideRouteLanesTest(self.ego_vehicles[0], route=route)

        # red_light_criterion = RunningRedLightTest(self.ego_vehicles[0])

        # stop_criterion = RunningStopTest(self.ego_vehicles[0])

        # blocked_criterion = ActorSpeedAboveThresholdTest(self.ego_vehicles[0],
        #                                                  speed_threshold=0.1,
        #                                                  below_threshold_max_time=90.0,
        #                                                  terminate_on_failure=True)

        criteria.append(completion_criterion)
        criteria.append(collision_criterion)
        criteria.append(route_criterion)
        # criteria.append(outsidelane_criterion)
        # criteria.append(red_light_criterion)
        # criteria.append(stop_criterion)
        # criteria.append(blocked_criterion)

        return criteria

    def post_scenario_behaviour(self):
        """
        Override this method to add post scenario behaviour to the actors
        """
        pass
    # NOTE: This method is not overridden to avoid removing the actors
    # def remove_all_actors(self):
    #     """
    #     Overriding this method to not remove all the actors.
    #     """
    #     pass