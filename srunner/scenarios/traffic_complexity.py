#!/usr/bin/env python

# Copyright (c) 2023 Okanagan Visualization & Interaction (OVI) Lab
# The University of British Columbia, BC, Canada
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Script used to run TOR scenario during the trial."""

import random
import py_trees
import json
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (SetInitSpeed,
                                                                      ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      ChangeActorControl,
                                                                      ChangeAutoPilot,
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
        self._actor_init_speeds = [] # This will be used to store the correct velocities of the actors to be spawned for the scenario
        self._actor_final_speeds = [] # This will be used to store the final velocities when TOR is about to be triggered

        # Storing the No interference transform. NOTE: This is only for town04 and is hardcoded. This needs to be changed for other towns
        self.no_interference_transform = carla.Transform(carla.Location(x=-314.248413, y=-39.597336, z=12.272047), carla.Rotation())
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

        # Offset for storing how below ground the vehicle should be spawned initially
        offset = 200
        # Spawn all the other vehicle in their respective locations
        for actor in config.other_actors:
            print(f"Spawning vehicle {actor.model} underground")
            
            # Figure out on which lane the vehicle must be spawned
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
            vehicle_scenario_transform = carla.Transform(
                carla.Location(vehicle_waypoint.transform.location.x,
                               vehicle_waypoint.transform.location.y,
                               vehicle_waypoint.transform.location.z + 0.2),
                vehicle_waypoint.transform.rotation)
            self._actor_transforms.append(vehicle_scenario_transform)

            # NOTE: The speed is in km/h and needs to be converted to m/s
            self._actor_init_speeds.append(actor.init_speed * 5/18)
            self._actor_final_speeds.append(actor.final_speed * 5/18)
            
            # Manipulating the waypoint transform's z value to spawn in below ground
            vehicle_init_transform = carla.Transform(
                carla.Location(self.no_interference_transform.location.x,
                               self.no_interference_transform.location.y,
                               self.no_interference_transform.location.z - offset),
                self.no_interference_transform.rotation)
            offset += 50 # Increment the offset for the next vehicle

            is_lead_vehicle = actor.lane == "same" and actor.vehicle_offset > 0 and actor.role == "relevant" # NOTE: Not a robust way to check if the vehicle is a lead vehicle

            # Set the rolename to store all the information regarding its behaviour
            # NOTE: The reason why this is done so that other compoenents of scenario_runner can access the information regarding the vehicle
            rolename_dict = {"is_lead": is_lead_vehicle, "init_speed": actor.init_speed*5/18, "final_speed": actor.final_speed*5/18}

            # Spawn the vehicle at a random location
            vehicle = CarlaDataProvider.request_new_actor(model=actor.model, spawn_point=vehicle_init_transform, rolename=json.dumps(rolename_dict))
            vehicle.set_simulate_physics(enabled=False)

            # Add the vehicle to the actor list
            self._other_actors.append(vehicle)

            # Check if the vehicle is a lead vehicle (that has the slower speed)
            if is_lead_vehicle:
                self._lead_vehicle = vehicle

            # Sanity check if values were added to all the three lists
            if not (len(self._other_actors) == len(self._actor_transforms) == len(self._actor_init_speeds)):
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

        # Now build the behaviour tree
        root = py_trees.composites.Parallel("Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        # Setting all the actors transform and  velocity using sequence composite
        for _, (vehicle, vehicle_transform, vehicle_velocity) in enumerate(zip(self._other_actors, self._actor_transforms, self._actor_init_speeds)):
            print("Pytrees: [Vehicle: {}; Transform: {}; Velocity: {} m/s]".format(vehicle, vehicle_transform, vehicle_velocity))

            # Creating a sequence tree for each vehicle params
            vehicle_params_setter = py_trees.composites.Sequence(f"Vehicle Parameters Setter: for vehicle id: {vehicle}")

            # Setting the transform
            set_above_transform = ActorTransformSetter(vehicle, vehicle_transform) # Set the transform of the vehicle above ground
            vehicle_params_setter.add_child(set_above_transform)

            # Setting the velocity
            set_init_velocity = SetInitSpeed(vehicle, vehicle_velocity) # Set the target speed immediately
            vehicle_params_setter.add_child(set_init_velocity)

            # Now, set the auto agent for the vehicles so they can drive by themselves
            # NOTE: Using WaypointFollower is not ideal as we want something that immediately returns SUCCESS
            # NOTE: control_py_module=None will return NPC agent's local planner control module
            set_agent = WaypointFollower(actor=vehicle, target_speed=vehicle_velocity, name=f"Initial Waypoint Follower for {vehicle.id}")
            # set_agent = ChangeActorControl(actor=vehicle, control_py_module=None, args={"target_speed": vehicle_velocity, "max_throttle": 0.75, "max_brake": 0.75})
            vehicle_params_setter.add_child(set_agent)

            # Lastly, add the vehicle params setter to the root
            root.add_child(vehicle_params_setter)

        # Now, add parallel behaviour for take-over request
        take_over_executer = py_trees.composites.Sequence("Setting Up Scenario for TOR")

        # TODO: Create behaviour to send a message to AutoHive for task-interleaving period

        # Adding Ideal behaviour for 30 seconds to help driver prepare for TOR
        idle_for_driver = Idle(duration=10)
        take_over_executer.add_child(idle_for_driver)

        # Setting a parallel composite to change the speed of all the vehicles and run WaypointFollower
        initialize_take_over = py_trees.composites.Parallel("Execute Scenario and TOR", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        for vehicle, final_speed in zip(self._other_actors, self._actor_final_speeds):
            # Creating a sequence to set speed and run WaypointFollower
            change_vehicle_speed = py_trees.composites.Sequence(f"Change Speed for {vehicle.id}")

            # Change the speed instantly first
            change_speed = SetInitSpeed(vehicle, final_speed)
            change_vehicle_speed.add_child(change_speed)

            # Now, change the current agent's target speed by cleverly using WaypointFollower
            change_target_speed = WaypointFollower(actor=vehicle, target_speed=final_speed, name=f"Scenario Waypoint Follower for {vehicle.id}")
            change_vehicle_speed.add_child(change_target_speed)

            # Lastly, add the change_vehicle_speed to the parallel composite
            initialize_take_over.add_child(change_vehicle_speed)

        # Setup ego vehicle behaviour for the TOR
        ego_vehicle_behaviour = py_trees.composites.Sequence("Ego Vehicle Behaviour for TOR")

        # TODO: Add behaviour to send a message to AutoHive for issuing a TOR

        # Turning on autopilot for several seconds. NOTE that the automation will turn off if driver gives input
        # NOTE: In order to do this the ego vehicle control first needs to be set to ExternalControl
        extenral_control = ChangeActorControl(actor=self.ego_vehicles[0], control_py_module=None, args=None)
        ego_vehicle_behaviour.add_child(extenral_control)
        post_tor_autopilot_on = ChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters={"auto_lane_change": False, "ignore_vehicles_percentage": 100, "max_speed": 100})
        # ego_vehicle_behaviour.add_child(post_tor_autopilot_on)

        # Provide automation for several seconds
        wait_for_driver_to_respond = Idle(duration=4)
        ego_vehicle_behaviour.add_child(wait_for_driver_to_respond)

        # Turn off autopilot after a time limit, if the driver has not responded already
        post_tor_autopilot_off = ChangeAutoPilot(actor=self._lead_vehicle, activate=False)
        ego_vehicle_behaviour.add_child(post_tor_autopilot_off)

        # Now, add the ego vehicle behaviour to take_over_executer parallel composite
        initialize_take_over.add_child(ego_vehicle_behaviour)

        # Lastly, add the initialize_take_over to the take_over_executer sequence
        take_over_executer.add_child(initialize_take_over)

        # Lastly, add the take over executer to the root
        root.add_child(take_over_executer)
        
        # Lastly, return the root
        return root
            
    def _create_test_criteria(self):
        """
        Setup the evaluation criteria for triggerring traffic complexity scenario
        """
        # Test if all the vehicles are spawned
        pass