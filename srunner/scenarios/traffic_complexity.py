#!/usr/bin/env python

# Copyright (c) 2023 Okanagan Visualization & Interaction (OVI) Lab
# The University of British Columbia, BC, Canada
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Script used to run TOR scenario during the trial."""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (SetInitSpeed,
                                                                      KeepVelocity,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

class TrafficComplexity(BasicScenario):
    """
    This scenario triggers a pre-alert, and also informs the driver about it.
    The pre-alert will run for 30 seconds after which a TOR is triggered.
    Some documentation on NewScenario
    :param world is the CARLA world
    :param ego_vehicles is a list of ego vehicles for this scenario
    :param config is the scenario configuration (ScenarioConfiguration)
    :param randomize can be used to select parameters randomly (optional, default=False)
    :param debug_mode can be used to provide more comprehensive console output (optional, default=False)
    :param criteria_enable can be used to disable/enable scenario evaluation based on test criteria (optional, default=True)
    :param timeout is the overall scenario timeout (optional, default=60 seconds)
    """

    # some ego vehicle parameters
    # some parameters for the other vehicles

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Initialize all parameters required for triggerring traffic complexity scenario
        """
        self._map = CarlaDataProvider.get_map()
        self._dreyevr_init_waypoint = self._map.get_waypoint(ego_vehicles[0].get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    

        # Call constructor of BasicScenario
        super(TrafficComplexity, self).__init__(
          "TrafficComplexity",
          ego_vehicles,
          config,
          world,
          debug_mode,
          criteria_enable=criteria_enable)


    def _create_behavior(self):
        """
        Setup the behavior for triggerring traffic complexity scenario
        """

        # NOTE: Spawn the vehicle either in the lane specified or exact opposite lane
        inverse = random.choice([True, False])

        # Setting all the actors velocity using sequence composite
        velocity_setter = py_trees.composites.Sequence("Velocity Setter")

        # Spawn all the vehicle in their respective locations and speed
        for actor in self.config.other_actors:
            # Figure out on which lane the vehicle must be spawned
            lane = actor.lane
            if lane not in ["left", "right", "same"]:
                raise RuntimeError(f"Invalid lane {lane} for {actor.model} on {actor.lane} at {actor.vehicle_offset}")
            
            if inverse and lane != "same":
                lane = "left" if lane == "right" else "right"

            # Get the waypoint where the vehicle must be spawned
            vehicle_waypoint, waypoint_distance = get_waypoint_in_distance(self._dreyevr_init_waypoint, actor.vehicle_offset, True, lane)

            # Check if a junction is not interefering with the spawn location
            if abs(waypoint_distance - actor.vehicle_offset) >= 5.0:
                raise RuntimeError(f"No spawn point found for {actor.model} on {actor.lane} at {actor.vehicle_offset}")
            
            # Spawn the vehicle at the location
            vehicle = CarlaDataProvider.request_new_actor(actor.model, vehicle_waypoint.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

            # Now once spawned, set the vehicle's velocity
            set_init_velocity = SetInitSpeed(vehicle, actor.speed) # Set the target speed immediately
            velocity_setter.add_child(set_init_velocity)
            set_keep_velocity = KeepVelocity(vehicle, actor.speed, True) # Keep the velocity immediately
            velocity_setter.add_child(set_keep_velocity)



            
    def _create_test_criteria(self):
        """
        Setup the evaluation criteria for triggerring traffic complexity scenario
        """
        # Test if all the vehicles are spawned
        pass