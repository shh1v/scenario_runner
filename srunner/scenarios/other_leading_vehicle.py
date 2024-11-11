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
import numpy as np

class EgoVehicleSensorHandler:
    def __init__(self, world):
        self.world = world
        self.sensor = DReyeVRSensor(world)  # Assuming DReyeVRSensor is already set up

    def publish_and_print(self, data):
        self.sensor.update(data)

        # Header mapping to map data keys to specific headers in the CSV
        header_mapping = {
            "left_gaze_dir": ["leftEyeLocalForward_X", "leftEyeLocalForward_Y", "leftEyeLocalForward_Z"],
            "left_gaze_origin": ["leftEyeLocalOrigin_X", "leftEyeLocalOrigin_Y", "leftEyeLocalOrigin_Z"],
            "right_gaze_dir": ["rightEyeLocalForward_X", "rightEyeLocalForward_Y", "rightEyeLocalForward_Z"],
            "right_gaze_origin": ["rightEyeLocalOrigin_X", "rightEyeLocalOrigin_Y", "rightEyeLocalOrigin_Z"],
            "right_pupil_posn": ["rightEyePupilPosition_X", "rightEyePupilPosition_Y"],
            "left_pupil_posn": ["leftEyePupilPosition_X", "leftEyePupilPosition_Y"],
            "gaze_dir": ["combinedEyeLocalForward_X", "combinedEyeLocalForward_Y", "combinedEyeLocalForward_Z"],
            "gaze_origin": ["combinedEyeLocalOrigin_X", "combinedEyeLocalOrigin_Y", "combinedEyeLocalOrigin_Z"],
            "framesequence": ["frameSequence"],
            "gaze_valid": ["isCombinedEyeGazeRayValid"],
            "left_eye_openness": ["leftEyeOpenness"],
            "left_eye_openness_valid": ["leftEyeOpennessReadSuccess"],    
            "left_gaze_valid": ["isLeftEyeGazeRayValid"],
            "left_pupil_diam": ["leftEyePupilDiameter"],
            "right_eye_openness": ["rightEyeOpenness"],
            "right_eye_openness_valid": ["rightEyeOpennessReadSuccess"],
            "right_gaze_valid": ["isRightEyeGazeRayValid"],
            "right_pupil_diam": ["rightEyePupilDiameter"],
            "timestamp": ["systemTimestamp(ms)"],
            "timestamp_device": ["deviceTimestamp(ms)"],
        }

        # Open the CSV file in append mode
        with open("sensor_data.csv", "a+", newline='') as f:
            writer = csv.writer(f)

            # Move to the start of the file and check if it's empty
            f.seek(0)
            if f.read(1):
                # If file is not empty, read the header
                f.seek(0)
                reader = csv.reader(f)
                header_row = next(reader)
            else:
                # If file is empty, dynamically create the header row
                header_row = ["Timestamp"]
                for key in self.sensor.data.keys():
                    if key in header_mapping:
                        header_row.extend(header_mapping[key])  # Use mapped headers
                    else:
                        header_row.append(key)  # Use original key as header if no mapping
                writer.writerow(header_row)  # Write the header row to the file

            # Initialize a dictionary to store values for each header
            row_dict = {header: "" for header in header_row}
            row_dict["Timestamp"] = int(datetime.now().timestamp())  # Set the timestamp

            # Populate the row dictionary with data from self.sensor.data
            for key, value in self.sensor.data.items():
                if key in header_mapping:
                    # If the key has mapped headers, assign values accordingly
                    headers = header_mapping[key]
                    if isinstance(value, (list, np.ndarray)) and len(value) == len(headers):
                        for h, v in zip(headers, value):
                            row_dict[h] = v
                    else:
                        # Handle single value cases by repeating if necessary
                        row_dict[headers[0]] = value
                else:
                    # If no mapping, store the value under its original key
                    row_dict[key] = value

            # Ensure values are in the correct order according to the header row
            row = [row_dict[header] for header in header_row]

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
