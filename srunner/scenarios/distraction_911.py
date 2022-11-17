#!/usr/bin/env python

"""
Distraction Mansion

This scenario has the (human driven) ego vehicle drive through the urban neighbourhood in Town03 twice, on the first go around nothing is
out of the ordinary, but on the 2nd loop there are several police cars, firetrucks, and an ambulance in the center mansion (as if something 
interesting happened, hence a 911 call) and there are crowds of people around so a pedestrian might jaywalk 
"""

import os, sys
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    RunFunction,
    WaitForSeconds,
    AccelerateToVelocity,
    StopVehicle,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CollisionTest,
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToLocation,
    DriveDistance,
)
from srunner.scenarios.basic_scenario import BasicScenario


class Distraction911(BasicScenario):

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

        assert (
            CarlaDataProvider.get_map().name == "Carla/Maps/Town01"
        )  # only Town01 supported!

        assert len(ego_vehicles) == 1
        self.ego_vehicle = ego_vehicles[0]
        assert "dreyevr" in self.ego_vehicle.type_id
        self.jaywalker = None

        super(Distraction911, self).__init__(
            "Distraction911",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _initialize_actors(self, config):

        police_idx = 30
        homeowner_idx = 5

        # fmt: off
        self.distractions = {
            # key: (vehicle model, vehicle pos [x, y, z], vehicle rot [roll, pitch, yaw]
            "policecar1": ("vehicle.dodge.charger_police", [152.1, 37.4, 1], [0, 0, 90]),
            "policecar2": ("vehicle.dodge.charger_police", [150.5, 29, 1], [0, 0, 120]),
            "ambulance": ("vehicle.ford.ambulance", [149.3, 24.0, 1], [0, 0, 0]),
            "firetruck": ("vehicle.carlamotors.firetruck", [150.9, 14.0, 0], [0, 0, 100]),
            "policeman1": (f"walker.pedestrian.{police_idx:04d}", [149.5, 38, 10.3], [0, 0, 170]),
            "policeman2": (f"walker.pedestrian.{police_idx:04d}", [149.4, 38.89, 10.3], [0, 0, 190]),
            "homeowner": (f"walker.pedestrian.{homeowner_idx:04d}", [148.5, 38.4, 10.3], [0, 0, 370]),
        }
        # fmt: on

        if self.jaywalker is None:
            try:
                # spawn jaywalker next to ambulance
                jaywalk_spawn_loc = carla.Location(*self.distractions["ambulance"][1])
                jaywalk_spawn_loc.x += 0.5
                jaywalk_spawn_loc.y -= (
                    1.5  # left of the ambulance (behind, so ego can't see)
                )
                jaywalk_spawn_loc.z = 0

                self.jaywalker = CarlaDataProvider.request_new_actor(
                    "walker.pedestrian.0009",
                    carla.Transform(jaywalk_spawn_loc, carla.Rotation()),
                )
            except Exception as e:
                print(f"unable to spawn jaywalker -- {e}")

        # spawn 911 distractions halfway through the route!
        # self._spawn_911()

    def _spawn_911(self):

        for name, spawn_data in self.distractions.items():
            model, pos, rot = spawn_data

            try:
                actor = CarlaDataProvider.request_new_actor(
                    model=model,
                    spawn_point=carla.Transform(
                        carla.Location(*pos),
                        carla.Rotation(roll=rot[0], pitch=rot[1], yaw=rot[2]),
                    ),
                )
                self.other_actors.append(actor)
            except Exception as e:
                print(f"unable to spawn actor: {name} -- {e}")
        print("spawned 911 distraction!")

    def _create_behavior(self):

        distance_thresh_jaywalk = 20  # meters to trigger jaywalk

        spawn_distractions = py_trees.composites.Sequence()
        spawn_distractions.add_child(
            InTriggerDistanceToLocation(
                actor=self.ego_vehicle,
                # middle of the route (check waypoints in xml file)
                target_location=carla.Location(x=334.8, y=14.9, z=0.0),
                distance=distance_thresh_jaywalk,  # distance threshold
                name=f"Waiting for ego to get close to middle of route to spawn distractions",
            )
        )
        spawn_distractions.add_child(
            RunFunction(function=self._spawn_911, args=())
        )

        jaywalk_sequence = py_trees.composites.Sequence()
        jaywalk_sequence.add_child(
            InTriggerDistanceToLocation(
                actor=self.ego_vehicle,
                target_location=carla.Location(*self.distractions["ambulance"][1]),
                distance=distance_thresh_jaywalk,  # distance threshold
                name=f"Waiting for ego to get close to distraction to trigger jaywalker",
            )
        )

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
                    distance=9,  # meters
                    name="walker runs for a while",
                )
            )
            jaywalk_sequence.add_child(
                StopVehicle(actor=self.jaywalker, brake_value=1.0, name="walker stop")
            )

        sequence = py_trees.composites.Sequence(
            name="Distraction911: Sequence Behaviour",
            children=[spawn_distractions, jaywalk_sequence],
        )
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
