import py_trees
import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorDestroy
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance
from DReyeVR_utils import DReyeVRSensor
import json
import numpy as np
import csv
from datetime import datetime

class EgoVehicleSensorHandler:
    def __init__(self, world):
        self.world = world
        self.sensor = DReyeVRSensor(world)  # Assuming DReyeVRSensor is already set up

    def publish_and_print(self, data):
    self.sensor.update(data)

    # Prepare data for CSV output
    with open("sensor_data.csv", "a", newline='') as f:  # Use "a" to append data
        writer = csv.writer(f)
        desired_header = [
            "brake_input", "camera_location", "camera_rotation", "current_gear_input", 
            "focus_actor_dist", "focus_actor_name", "focus_actor_pt", "frame", "frame_number", 
            "framesequence", "combinedEyeLocalForward_X", "combinedEyeLocalForward_Y", 
            "combinedEyeLocalForward_Z", "combinedEyeLocalOrigin_X", "combinedEyeLocalOrigin_Y", 
            "combinedEyeLocalOrigin_Z", "leftEyeLocalForward_X", "leftEyeLocalForward_Y", "leftEyeLocalForward_Z",  # Split left_gaze_dir into X, Y, Z
            "leftEyeLocalOrigin_X", "leftEyeLocalOrigin_Y", "leftEyeLocalOrigin_Z",  # Split left_gaze_origin into X, Y, Z
            "isLeftEyeGazeRayValid", "gaze_vergence", "handbrake_input", 
            "leftEyeOpenness", "leftEyeOpennessReadSuccess", # Split right_gaze_dir and origin
            "leftEyePupilDiameter", "leftEyePupilPosition_X", "leftEyePupilPosition_Y","left_pupil_posn_valid", 
            "rightEyeOpenness", "rightEyeOpennessReadSuccess", "rightEyeLocalForward_X", "rightEyeLocalForward_Y", 
            "rightEyeLocalForward_Z", "rightEyeLocalOrigin_X", "rightEyeLocalOrigin_Y", "rightEyeLocalOrigin_Z", 
            "isRightEyeGazeRayValid", "rightEyePupilDiameter", "rightEyePupilPosition_X", 
            "rightEyePupilPosition_Y", "right_pupil_posn_valid ", "steering_input", "throttle_input", 
            "systemTimestamp(ms)", "timestamp_carla", "deviceTimestamp(ms)", "timestamp_stream", "transform"
        ]

        # Write the header if the file is empty
        if f.tell() == 0:
            writer.writerow(["Timestamp"] + desired_header)  # Add header with the desired columns
        
        # Prepare row with timestamp and sensor data
        timestamp = int(datetime.now().timestamp())
        row = [timestamp]
        
        for key in self.sensor.data.keys():
            value = self.sensor.data[key]

            # If the value is a 3D vector (like left_gaze_dir, right_gaze_dir, left_gaze_origin, or right_gaze_origin), split it
            if isinstance(value, str) and value.startswith('[') and value.endswith(']'):
                # Parse the string into a list of floats
                value = [float(v) for v in value[1:-1].split(',')]
                
                if key == "left_gaze_dir":
                    row.extend(value)  # Adding left_gaze_dir_X, left_gaze_dir_Y, left_gaze_dir_Z to the row
                elif key == "left_gaze_origin":
                    row.extend(value)  # Adding left_gaze_origin_X, left_gaze_origin_Y, left_gaze_origin_Z to the row
                elif key == "right_gaze_dir":
                    row.extend(value)  # Adding right_gaze_dir_X, right_gaze_dir_Y, right_gaze_dir_Z to the row
                elif key == "right_gaze_origin":
                    row.extend(value)  # Adding right_gaze_origin_X, right_gaze_origin_Y, right_gaze_origin_Z to the row
                elif key == "left_pupil_posn":
                    # Split the 2 elements for left_pupil_posn into the correct components
                    row.extend(value) 
                elif key == "right_pupil_posn":
                    # Split the 2 elements for left_pupil_posn into the correct components
                    row.extend(value) 
                elif key == "gaze_dir":
                    # Split the 2 elements for left_pupil_posn into the correct components
                    row.extend(value) 
                elif key == "gaze_origin":
                    # Split the 2 elements for left_pupil_posn into the correct components
                    row.extend(value) 
                else:
                    row.append(value)  # For other fields that aren't gaze-related
            else:
                # Convert numpy arrays to lists for better readability
                if isinstance(value, np.ndarray):
                    value = value.tolist()
                row.append(value)

        # Write the row to the CSV file
        writer.writerow(row)


    def listen_to_sensor(self):
        self.sensor.ego_sensor.listen(self.publish_and_print)



class OtherLeadingVehicle(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=300):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._spawn_offset = 20
        self.timeout = timeout
        
        # Initialize the EgoVehicleSensorHandler
        self.sensor_handler = EgoVehicleSensorHandler(world)
        self.sensor_handler.listen_to_sensor()  # Start listening

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
