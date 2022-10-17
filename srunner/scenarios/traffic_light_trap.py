#!/usr/bin/env python

"""
Traffic light trap:

This scenario has the (human driven) ego vehicle be "trapped" by two yellow->red lights that turn *just* when 
the EgoVehicle is about to make it. This aggravates them since it occurs multiple times, making it more likely 
for the human driver to run the last one. 
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter,
    StopVehicle,
    LaneChange,
    WaypointFollower,
    Idle,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
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

        # locations for the traffic lights
        self.trigger_locations = config.trigger_points
        assert len(self.trigger_locations) >= self.trap_length

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

    def _get_traffic_light(self, location):
        _traffic_light = CarlaDataProvider.get_next_traffic_light_by_location(
            location, False
        )
        if _traffic_light is None:
            print(f"No traffic light for the given location of {location} found")
            sys.exit(-1)
        return _traffic_light

    def _initialize_actors(self, config):

        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

        ego_vehicle_start, _ = get_waypoint_in_distance(
            self._reference_waypoint, self._fast_vehicle_distance
        )
        self.ego_vehicle_start = carla.Transform(
            carla.Location(
                ego_vehicle_start.transform.location.x,
                ego_vehicle_start.transform.location.y,
                ego_vehicle_start.transform.location.z + 1,
            ),
            ego_vehicle_start.transform.rotation,
        )

        # traffic light actors
        self.traffic_light = [
            self._get_traffic_light(self.trigger_locations[i])
            for i in range(self.trap_length)
        ]
        # traffic light initial state
        self.traffic_light[0].set_state(carla.TrafficLightState.green)
        self.traffic_light[1].set_state(carla.TrafficLightState.green)
        self.traffic_light[2].set_state(carla.TrafficLightState.yellow)
        self.traffic_light[3].set_state(carla.TrafficLightState.red)

    def _create_behavior(self):

        sequence = py_trees.composites.Sequence(
            "Traffic light trap: Sequence Behaviour"
        )

        ego_in_ranges = [
            InTriggerDistanceToVehicle(
                self.trigger_locations[i],
                self.ego_vehicle,
                50,  # distance threshold
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

        # other_traffic_hack = TrafficLightStateSetterWithTime(
        #     actor=self._traffic_light_other,
        #     state=carla.TrafficLightState.Green,
        #     duration=self.timeout,
        #     name="Change opposite light to green S8",
        # )

        start_scenario = py_trees.composites.Sequence()
        for i in range(self.trap_length):
        start_scenario.add_child(ego_in_range)
        start_scenario.add_child(ego_traffic_hack)
        start_scenario.add_child(other_traffic_hack)

        sequence.add_child(start_scenario)
        sequence.add_child(parallel_behaviour_root)
        # sequence.add_child(ActorDestroy(self.ego_vehicle)) # don't destroy ego!

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicle)

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
