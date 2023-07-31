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

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Initialize all parameters required for triggerring traffic complexity scenario
        """
        print("Scenario: Initializing Traffic Complexity Scenario")
              
        self._world = world
        self._ego_vehicles = ego_vehicles
        self._lead_vehicle = None
        self._config = config
        print("other:", config.other_actors)
        print("ego:", ego_vehicles)
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

        print("Scenario: Initializing Traffic Complexity Behavior")

        # NOTE: Spawn the vehicle either in the lane specified or exact opposite lane
        inverse = random.choice([True, False])

        # Setting all the actors velocity using sequence composite
        velocity_setter = py_trees.composites.Sequence("Velocity Setter")

        # Spawn all the other vehicle in their respective locations and speed
        for actor in self._config.other_actors:
            # Figure out on which lane the vehicle must be spawned
            print("Spawning {} on {} at {}".format(actor.model, actor.lane, str(actor.vehicle_offset)))
            lane = actor.lane
            if lane not in ["left", "right", "same"]:
                raise RuntimeError(f"Invalid lane {lane} for {actor.model} on {actor.lane} at {actor.vehicle_offset}")
            
            if inverse and lane != "same":
                lane = "left" if lane == "right" else "right"
            
            # Check if the vehicle is a lead vehicle (that has the slower speed)
            if actor.lane == "same" and actor.vehicle_offset > 0 and actor.role == "relevant":
                self._lead_vehicle = actor

            # Get the waypoint where the vehicle must be spawned
            vehicle_waypoint, waypoint_distance = get_waypoint_in_distance(self._dreyevr_init_waypoint, actor.vehicle_offset, False, lane)

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
        
        # Make sure that the lead vehicle is spawned
        if self._lead_vehicle is None:
            raise RuntimeError("Lead vehicle not spawned. Please verify the scenario configuration.")

        # Once the actors are spawned and their velocity is set, add the trigger behaviour
        # NOTE: The Idle behaviour can also be used instead of trigger behaviour but the time budget as a confounding mught be an issue
        TOR_trigger_condition = py_trees.composites.Sequence("Take-Over Request Trigger Condition")
        trigger_distance = (self._ego_vehicles[0].get_velocity().length() - self._lead_vehicle.get_velocity().length()) * 10.0
        trigger_when_close = InTriggerDistanceToVehicle(self._lead_vehicle, self._ego_vehicles[0], distance=trigger_distance, name="Trigger TOR when close")
        TOR_trigger_condition.add_child(trigger_when_close)

        # TODO: Once the TOR is triggered, communicate this to CARLA

        # Now build the behaviour tree
        root = py_trees.composites.Parallel("Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(velocity_setter)
        root.add_child(TOR_trigger_condition)

        return root
            
    def _create_test_criteria(self):
        """
        Setup the evaluation criteria for triggerring traffic complexity scenario
        """
        # Test if all the vehicles are spawned
        pass