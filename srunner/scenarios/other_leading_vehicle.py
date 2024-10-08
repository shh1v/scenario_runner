import py_trees
import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (WaypointFollower, ActorTransformSetter, ActorDestroy)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

class OtherLeadingVehicle(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=120):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        
        # Define distance in front of the ego vehicle
        self._spawn_offset = 20  # Distance in meters
        self._first_vehicle_speed = 45 / 3.6  # Speed in m/s

        # Store the timeout value
        self.timeout = timeout  # Ensure this is initialized

        super(OtherLeadingVehicle, self).__init__("VehicleLeadingScenario", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # Calculate the spawn point for the leading vehicle
        leading_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._spawn_offset)
        leading_vehicle_transform = carla.Transform(leading_vehicle_waypoint.transform.location, leading_vehicle_waypoint.transform.rotation)
        
        # Spawn the leading vehicle
        leading_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', leading_vehicle_transform)
        self.other_actors.append(leading_vehicle)

        # Set the leading vehicle to follow the route
        leading_vehicle.set_autopilot(True)

    def _create_behavior(self):
        # Create the behavior tree
        driving_in_same_direction = py_trees.composites.Parallel("DrivingTowardsIntersection", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        
        # Make the leading vehicle follow the waypoints
        leading_vehicle_behavior = WaypointFollower(self.other_actors[0], self._first_vehicle_speed, avoid_collision=True)

        driving_in_same_direction.add_child(leading_vehicle_behavior)

        # Drive the ego vehicle a certain distance
        ego_drive_distance = DriveDistance(self.ego_vehicles[0], 100)  # Adjust distance as needed
        sequence = py_trees.composites.Sequence("Scenario behavior")
        sequence.add_child(driving_in_same_direction)
        sequence.add_child(ego_drive_distance)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        # Add criteria if needed
        pass

    def __del__(self):
        self.remove_all_actors()
