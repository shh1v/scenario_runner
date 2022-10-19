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
    InTriggerDistanceToLocation,
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
        self.other_actors = []

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
        self.traffic_light_locns = [
            # [x, y, z] according to editor
            carla.libcarla.Location(*[77.48, 4.83, 0.0]),
            carla.libcarla.Location(*[85.3, 45.78, 0.0]),
            carla.libcarla.Location(*[85, 120, 0.0]),
            carla.libcarla.Location(*[85, 185.75, 0.0]),
        ]

        assert len(self.traffic_light_locns) == self.trap_length

        self.traffic_light = [
            CarlaDataProvider.get_next_traffic_light_by_location(locn)
            for locn in self.traffic_light_locns
        ]

        # traffic light initial state (all green forever until vehicle approaches)
        for i in range(self.trap_length):
            self.traffic_light[i].set_state(carla.TrafficLightState.Green)
            self.traffic_light[i].set_green_time(1000)

    def _create_behavior(self):

        distance_thresh_yellow = 40  # meters to trigger yellow
        distance_thresh_red = 20  # meters to trigger red

        ego_in_ranges_yellow = [
            InTriggerDistanceToLocation(
                actor=self.ego_vehicle,
                target_location=self.traffic_light_locns[i],
                distance=distance_thresh_yellow,  # distance threshold
                name=f"Waiting for ego to get close to traffic light (yellow) {i}",
            )
            for i in range(self.trap_length)
        ]

        ego_traffic_hack_yellow = [
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

        ego_in_ranges_red = [
            InTriggerDistanceToLocation(
                actor=self.ego_vehicle,
                target_location=self.traffic_light_locns[i],
                distance=distance_thresh_red,  # distance threshold
                name=f"Waiting for ego to get close to traffic light (RED) {i}",
            )
            for i in range(self.trap_length)
        ]

        ego_traffic_hack_red = [
            TrafficLightStateSetterWithTime(
                actor=self.traffic_light[i],
                state=carla.TrafficLightState.Red,
                duration=5000,
                name="Change ego light to Red for a short while",
            )
            for i in range(self.trap_length)
        ]

        sequence = py_trees.composites.Sequence(
            "Traffic light trap: Sequence Behaviour"
        )
        start_scenario = py_trees.composites.Sequence()
        for i in range(self.trap_length):
            start_scenario.add_child(ego_in_ranges_yellow[i])
            start_scenario.add_child(ego_traffic_hack_yellow[i])
            start_scenario.add_child(ego_in_ranges_red[i])
            start_scenario.add_child(ego_traffic_hack_red[i])

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
