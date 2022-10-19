#!/usr/bin/env python

"""
Traffic light trap:

This scenario has the (human driven) ego vehicle be "trapped" by two yellow->red lights that turn *just* when 
the EgoVehicle is about to make it. This aggravates them since it occurs multiple times, making it more likely 
for the human driver to run the last one. 
"""

import os, sys
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    TrafficLightStateSetterWithTime,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CollisionTest,
    RunningRedLightTest,
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToVehicle,
    StandStill,
)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class TrafficLightTrap(BasicScenario):

    """
    This is a single ego vehicle scenario
    """

    timeout = 12000  # ms

    def __init__(
        self,
        world,
        ego_vehicles,
        config,
        randomize=False,
        debug_mode=False,
        criteria_enable=True,
        timeout=600,
    ):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location
        )

        # number of traffic lights to trap ego in
        self.trap_length: int = 4

        assert len(ego_vehicles) == 1
        self.ego_vehicle = ego_vehicles[0]
        assert "dreyevr" in self.ego_vehicle.type_id

        super(TrafficLightTrap, self).__init__(
            "ChangeLane",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _initialize_actors(self, config):

        # traffic light actors
        traffic_light_locns = [
            # [x, y, z] according to editor
            carla.libcarla.Location(*[77.48, 4.83, 0.0]),
            carla.libcarla.Location(*[85.3, 45.78, 0.0]),
            carla.libcarla.Location(*[85, 120, 0.0]),
            carla.libcarla.Location(*[85, 185.75, 0.0]),
        ]

        self.traffic_light = [
            CarlaDataProvider.get_next_traffic_light_by_location(locn)
            for locn in traffic_light_locns
        ]

        # traffic light initial state
        self.traffic_light[0].set_state(carla.TrafficLightState.Green)
        self.traffic_light[1].set_state(carla.TrafficLightState.Green)
        self.traffic_light[2].set_state(carla.TrafficLightState.Yellow)
        self.traffic_light[3].set_state(carla.TrafficLightState.Red)

    def _create_behavior(self):

        ego_in_ranges = [
            InTriggerDistanceToVehicle(
                self.traffic_light[i],
                self.ego_vehicle,
                10,  # distance threshold
                name=f"Waiting for ego to get close to traffic light {i}",
            )
            for i in range(self.trap_length)
        ]

        ego_traffic_hacks = [
            TrafficLightStateSetterWithTime(
                actor=self.traffic_light[i],
                state=carla.TrafficLightState.Yellow,
                duration=1000
                if i < self.trap_length - 1
                else 5000,  # more time on the last one
                name="Change ego light to yellow for a short while",
            )
            for i in range(self.trap_length)
        ]

        sequence = py_trees.composites.Sequence(
            "Traffic light trap: Sequence Behaviour"
        )
        start_scenario = py_trees.composites.Sequence()
        start_scenario.add_child(ChangeAutoPilot(self.ego_vehicle, False))
        for i in range(self.trap_length):
            start_scenario.add_child(ego_in_ranges[i])
            start_scenario.add_child(ego_traffic_hacks[i])

        sequence.add_child(start_scenario)
        # sequence.add_child(ActorDestroy(self.ego_vehicle)) # don't destroy ego!

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicle)
        traffic_criterion = RunningRedLightTest(self.ego_vehicle)

        criteria.append(collision_criterion)
        criteria.append(traffic_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
