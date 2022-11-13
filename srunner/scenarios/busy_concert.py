#!/usr/bin/env python

"""
Busy concert:

This scenario has the ego vehicle drive through a busy section of the map
"""

import os, sys
import py_trees
import carla
import random

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


class BusyConcert(BasicScenario):

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

        super(BusyConcert, self).__init__(
            "BusyConcert",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _initialize_actors(self, config):

        random.seed(CarlaDataProvider.get_rng_seed(None))

        num_walkers = 10
        spawn_loc = []
        spawn_loc += [[158.5 - 5 * i, 298.2, 1.2] for i in range(20)]
        spawn_loc += [[156.5 - 5 * i, 295.6, 1.2] for i in range(12)]
        spawn_loc += [[183.3 - 5 * i, 310.3, 1.2] for i in range(25)]
        random.shuffle(spawn_loc)

        for i in range(num_walkers):
            walker = self._spawn_actor(
                idx=random.randint(1, 30), location=spawn_loc[i % len(spawn_loc)], rotation=None
            )
            if walker is not None:
                walker.apply_tag("DummyWalker") # start the DReyeVR walker policy
                self.other_actors.append(walker)
        print(f"Successfully spawned {len(self.other_actors)} actors")

    def _spawn_actor(self, idx: int, location: list, rotation: float = None):
        # location = [x,y,z]
        # rotation = Optional[yaw] (else None => random)
        walker = None
        try:
            if rotation is None:
                rotation = random.randint(0, 360)  # some random dir
            walker = CarlaDataProvider.request_new_actor(
                f"walker.pedestrian.{idx:04d}",
                carla.Transform(
                    carla.Location(*location),
                    carla.Rotation(roll=0, pitch=0, yaw=rotation),
                ),
            )
            print(f"Spawned {walker}")
        except Exception as e:
            print(f"unable to spawn actor: {e}")
        return walker

    def _create_behavior(self):

        distance_thresh: float = 10  # meters
        jaywalk_location_spawn = [103.1, 299.6, 1.1]  # where on the map to spawn
        jaywalk_rotation_spawn = 90

        jaywalk_sequence = py_trees.composites.Sequence()
        jaywalk_sequence.add_child(
            InTriggerDistanceToLocation(
                actor=self.ego_vehicle,
                target_location=carla.Location(*jaywalk_location_spawn),
                distance=distance_thresh,  # distance threshold
                name=f"Waiting for ego to get close to jaywalker",
            )
        )

        if self.jaywalker is None:
            self.jaywalker = self._spawn_actor(
                idx=9, location=jaywalk_location_spawn, rotation=jaywalk_rotation_spawn
            )

        if self.jaywalker is not None:
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
            name="Busy Concert: Parallel Behaviour",
            children=[jaywalk_sequence],
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

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
