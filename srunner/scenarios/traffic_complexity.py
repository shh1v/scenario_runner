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
                                                                      ActorTransformSetter,
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

        # Some variables helpful for the scenario implementation
        self._world = world
        self._ego_vehicles = ego_vehicles
        self._config = config
        self._map = CarlaDataProvider.get_map()

        # Some variables to needed for the pytrees
        self._lead_vehicle = None # This will be to used for the trigger condition
        self._other_actors = [] # This will be used to store all the other actors spawned in the scenario
        self._actor_transforms = [] # This will be used to store the correct transforms of the actors to be spawned for the scenario
        self._actor_velocities = [] # This will be used to store the correct velocities of the actors to be spawned for the scenario

        print(self._config.trigger_points[0])
        # Call constructor of BasicScenario
        super(TrafficComplexity, self).__init__(
          "TrafficComplexity",
          ego_vehicles,
          config,
          world,
          debug_mode,
          criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Setup the behavior for triggerring traffic complexity scenario
        """

        # NOTE: Spawn the vehicle either in the lane specified or exact opposite lane
        inverse = random.choice([True, False])

        # Spawn all the other vehicle in their respective locations
        for actor in config.other_actors:
            # Figure out on which lane the vehicle must be spawned
            print("Spawning underground {} on {} at {}".format(actor.model, actor.lane, str(actor.vehicle_offset)))

            lane = actor.lane
            if lane not in ["left", "right", "same"]:
                raise RuntimeError(f"Invalid lane {lane} for {actor.model} on {actor.lane} at {actor.vehicle_offset}")
            
            if inverse and lane != "same":
                lane = "left" if lane == "right" else "right"

            # Get the waypoint where the vehicle must be spawned
            vehicle_waypoint, waypoint_distance = get_waypoint_in_distance(self._map.get_waypoint(config.trigger_points[0].location), actor.vehicle_offset, False, lane)

            # Check if a junction is not interefering with the spawn location
            if abs(abs(waypoint_distance) - abs(actor.vehicle_offset)) >= 5.0:
                raise RuntimeError(f"No spawn point found for {actor.model} on {actor.lane} at {actor.vehicle_offset}. Difference: {abs(waypoint_distance) - abs(actor.vehicle_offset)}")
            
            # Append the above ground vehicle transform and velocity to the list
            vehicle_above_transform = carla.Transform(
                carla.Location(vehicle_waypoint.transform.location.x,
                               vehicle_waypoint.transform.location.y,
                               vehicle_waypoint.transform.location.z + 1),
                vehicle_waypoint.transform.rotation)
            self._actor_transforms.append(vehicle_above_transform)
            self._actor_velocities.append(actor.speed)

            # Manipulating the waypoint transform's z value to spawn in below ground
            vehicle_below_transform = carla.Transform(
                carla.Location(vehicle_waypoint.transform.location.x,
                               vehicle_waypoint.transform.location.y,
                               vehicle_waypoint.transform.location.z - 500),
                vehicle_waypoint.transform.rotation)

            # Spawn the vehicle at a random location
            vehicle = CarlaDataProvider.request_new_actor(actor.model, vehicle_below_transform)
            vehicle.set_simulate_physics(enabled=False)

            # Add the vehicle to the actor list
            self._other_actors.append(vehicle)

            # Check if the vehicle is a lead vehicle (that has the slower speed)
            if actor.lane == "same" and actor.vehicle_offset > 0 and actor.role == "relevant":
                self._lead_vehicle = vehicle

            # Sanity check if values were added to all the three lists
            if not (len(self._other_actors) == len(self._actor_transforms) == len(self._actor_velocities)):
                raise RuntimeError("Error in adding the actor to the list")

        # Make sure that the lead vehicle is spawned
        if self._lead_vehicle is None:
            raise RuntimeError("Lead vehicle not spawned. Please verify the scenario configuration.")
        
        # Make sure that the lead vehicle is spawned
        if len(config.other_actors) != len(self._other_actors):
            raise RuntimeError("All the vehicles not spawned. Please verify the scenario configuration.")

    def _create_behavior(self):
        """
        Setup the behavior for triggerring traffic complexity scenario
        """

        # Setting all the actors transform and  velocity using sequence composite
        vehicle_params_setter = py_trees.composites.Sequence("Vehicle transform and velocity setter")

        for _, (vehicle, vehicle_transform, vehicle_velocity) in enumerate(zip(self._other_actors, self._actor_transforms, self._actor_velocities)):
            # Setting the transform
            set_above_transform = ActorTransformSetter(vehicle, vehicle_transform) # Set the transform of the vehicle above ground
            vehicle_params_setter.add_child(set_above_transform)

            # Setting the velocity
            set_init_velocity = SetInitSpeed(vehicle, vehicle_velocity) # Set the target speed immediately
            vehicle_params_setter.add_child(set_init_velocity)

        # Once the actors are spawned and their velocity is set, add the trigger behaviour
        # NOTE: The Idle behaviour can also be used instead of trigger behaviour but the time budget as a confounding mught be an issue
        TOR_trigger_condition = py_trees.composites.Sequence("Take-Over Request Trigger Condition")
        trigger_distance = 50 # TODO: Set this dynamically based on the speed of the lead vehicle
        trigger_when_close = InTriggerDistanceToVehicle(self._lead_vehicle, self._ego_vehicles[0], distance=trigger_distance, name="Trigger TOR when close")
        TOR_trigger_condition.add_child(trigger_when_close)

        # TODO: Once the TOR is triggered, communicate this to CARLA

        # Now build the behaviour tree
        root = py_trees.composites.Parallel("Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(vehicle_params_setter)
        root.add_child(TOR_trigger_condition)

        return root
            
    def _create_test_criteria(self):
        """
        Setup the evaluation criteria for triggerring traffic complexity scenario
        """
        # Test if all the vehicles are spawned
        pass