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
    TrafficLightStateSetter,
    WaitForSeconds,
    AccelerateToVelocity,
    StopVehicle,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CollisionTest,
    RunningRedLightTest,
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToLocation,
    DriveDistance,
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
        self._start_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self.other_actors = []

        # number of traffic lights to trap ego in
        self.trap_length: int = 4

        assert len(ego_vehicles) == 1
        self.ego_vehicle = ego_vehicles[0]
        assert "dreyevr" in self.ego_vehicle.type_id
        self.jaywalker = None

        super(TrafficLightTrap, self).__init__(
            "TrafficLightTrap",
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
            carla.libcarla.Location(*[101.98, -4.2, 0.0]),
            carla.libcarla.Location(*[85.3, 45.78, 0.0]),
            carla.libcarla.Location(*[85, 120, 0.0]),
            carla.libcarla.Location(*[85, 185.75, 0.0]),
        ]

        assert len(traffic_light_locns) == self.trap_length

        self.traffic_light = [
            CarlaDataProvider.get_next_traffic_light_by_location(locn)
            for locn in traffic_light_locns
        ]

        # # get all the subsequent traffic lights from the ego-vehicle's position
        # self.traffic_lights = [
        #     CarlaDataProvider.get_next_traffic_light_by_location(
        #         self._start_waypoint.transform.location
        #     )
        # ]
        # # TODO: these are NOT working (because the traffic light waypoints are immediately flagged as "is_intersection" which leads to early-out
        # # and we need to account for the soonest non-equal intersection with the traffic sign in the correct (us-facing) orientation)
        # for i in range(1, self.trap_length):
        #     self.traffic_lights.append(
        #         CarlaDataProvider.get_next_traffic_light(
        #             self.traffic_lights[i - 1], use_cached_location=False
        #         )
        #     )

        # traffic light initial state (all green forever until vehicle approaches)
        for light in self.traffic_light:
            light.set_state(carla.TrafficLightState.Green)
            light.set_green_time(1000)

    def _create_behavior(self):

        distance_thresh_yellow = 40  # meters to trigger yellow
        distance_thresh_red = 20  # meters to trigger red
        light_duration_long = 5  # seconds
        light_duration_short = 1  # seconds

        traffic_light_seq = py_trees.composites.Sequence()
        for i in range(self.trap_length):
            """Get close to light to turn it to yellow"""
            traffic_light_seq.add_child(
                InTriggerDistanceToLocation(
                    actor=self.ego_vehicle,
                    target_location=CarlaDataProvider._traffic_light_map[
                        self.traffic_light[i]
                    ].location,
                    distance=distance_thresh_yellow,  # distance threshold
                    name=f"Waiting for ego to get close to traffic light (yellow) {i}",
                )
            )
            """Set traffic light to yellow"""
            traffic_light_seq.add_child(
                TrafficLightStateSetter(
                    actor=self.traffic_light[i],
                    state=carla.TrafficLightState.Yellow,
                    name="Change ego light to yellow for a short while",
                )
            )
            """Get close to light to turn it to red"""
            traffic_light_seq.add_child(
                InTriggerDistanceToLocation(
                    actor=self.ego_vehicle,
                    target_location=CarlaDataProvider._traffic_light_map[
                        self.traffic_light[i]
                    ].location,
                    distance=distance_thresh_red,  # distance threshold
                    name=f"Waiting for ego to get close to traffic light (RED) {i}",
                )
            )
            """Set traffic light to red"""
            traffic_light_seq.add_child(
                TrafficLightStateSetter(
                    actor=self.traffic_light[i],
                    state=carla.TrafficLightState.Red,
                    name="Change ego light to Red for a decent while",
                )
            )
            """Wait some duration"""
            traffic_light_seq.add_child(WaitForSeconds(light_duration_long))
            """Set traffic light back to green"""
            traffic_light_seq.add_child(
                TrafficLightStateSetter(
                    actor=self.traffic_light[i],
                    state=carla.TrafficLightState.Green,
                    name="Change ego light to green for a decent while",
                )
            )

        """PEDESTRIAN JAYWALKING ON THE VERY LAST ONE"""
        jaywalk_sequence = py_trees.composites.Sequence()
        jaywalk_sequence.add_child(
            InTriggerDistanceToLocation(
                actor=self.ego_vehicle,
                target_location=CarlaDataProvider._traffic_light_map[
                    self.traffic_light[-1]
                ].location,
                distance=distance_thresh_red,  # distance threshold
                name=f"Waiting for ego to get close to traffic light (red) to trigger jaywalker {i}",
            )
        )

        if self.jaywalker is None:
            try:
                self.jaywalker = CarlaDataProvider.request_new_actor(
                    "walker.pedestrian.0009",
                    carla.Transform(
                        carla.Location(x=85.4, y=189.1, z=0.65), carla.Rotation()
                    ),
                )
            except:  # pylint: disable=bare-except
                print("unable to spawn actor")
        if self.jaywalker is not None:
            # target_locn = carla.Location(x=103, y=189.1, z=0.65)
            jaywalk_sequence.add_child(
                AccelerateToVelocity(
                    actor=self.jaywalker, throttle_value=1.0, target_velocity=5
                )
            )
            jaywalk_sequence.add_child(
                DriveDistance(
                    actor=self.jaywalker,
                    distance=10,  # meters
                    name="walker runs for a while",
                )
            )
            jaywalk_sequence.add_child(
                StopVehicle(actor=self.jaywalker, brake_value=1.0, name="walker stop")
            )

        sequence = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="Traffic light trap: Parallel Behaviour",
            children=[traffic_light_seq, jaywalk_sequence],
        )
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
