import py_trees
import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorDestroy
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

class OtherLeadingVehicle(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=300):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        
        self._spawn_offset = 20  # Distance in meters

        # Store the timeout value
        self.timeout = timeout  # Set to 300 seconds (5 minutes)

        super(OtherLeadingVehicle, self).__init__("VehicleLeadingScenario", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        leading_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._spawn_offset)
        leading_vehicle_transform = carla.Transform(leading_vehicle_waypoint.transform.location, leading_vehicle_waypoint.transform.rotation)
        
        # Spawn the leading vehicle
        leading_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', leading_vehicle_transform)
        self.other_actors.append(leading_vehicle)

        # Set the leading vehicle to autopilot mode
        leading_vehicle.set_autopilot(True)

    def _create_behavior(self):
        # Create the behavior tree
        sequence = py_trees.composites.Sequence("Scenario behavior")

        # Drive the ego vehicle a distance for 5 minutes (300 seconds)
        ego_drive_distance = DriveDistance(self.ego_vehicles[0], 3750)  # Adjusted distance for 5 minutes
        sequence.add_child(ego_drive_distance)

        # After driving, destroy the leading vehicle
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        # Add criteria if needed
        pass

    def __del__(self):
        self.remove_all_actors()
