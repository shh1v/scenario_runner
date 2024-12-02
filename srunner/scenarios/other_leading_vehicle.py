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
import logging

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
            timestamp_ms = int(datetime.now().timestamp() * 1000)

            # Set the timestamp in your row dictionary
            row_dict["Timestamp"] = timestamp_ms
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

import carla
import py_trees
import logging

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class OtherLeadingVehicle(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=600):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._spawn_offset = 20
        self.timeout = timeout

        super(OtherLeadingVehicle, self).__init__(
            "VehicleLeadingScenario",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )
        self.LOG_insert("file.log", "Starting scenario with Sebastian severity 0.80", logging.INFO)
        # Initialize the EgoVehicleSensorHandler
        self.sensor_handler = EgoVehicleSensorHandler(world)
        self.sensor_handler.listen_to_sensor()  # Start listening

    def LOG_insert(self, file, text, level):
        infoLog = logging.FileHandler(file)
        infoLog.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
        logger = logging.getLogger(file)
        logger.setLevel(level)
        if not logger.handlers:
           logger.addHandler(infoLog)
           if (level == logging.INFO):
               logger.info(text)
           if (level == logging.ERROR):
               logger.error(text)
           if (level == logging.WARNING):
                logger.warning(text)
    
        infoLog.close()
        logger.removeHandler(infoLog)
    
        return
    def _initialize_actors(self, config):
        leading_vehicle_waypoint, _ = self._get_waypoint_in_distance(self._reference_waypoint, self._spawn_offset)
        leading_vehicle_transform = carla.Transform(
            leading_vehicle_waypoint.transform.location, leading_vehicle_waypoint.transform.rotation
        )

        # Spawn the leading vehicle
        leading_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', leading_vehicle_transform)
        self.other_actors.append(leading_vehicle)

        # Set the leading vehicle to autopilot mode
        leading_vehicle.set_autopilot(True)

    def _create_behavior(self):
        # Create the behavior tree
        sequence = py_trees.composites.Sequence("Scenario Behavior")

        # Add a leading vehicle stopping logic based on ego vehicle proximity
        def stop_leading_vehicle_if_far():
            """
            Stops the leading vehicle if the ego vehicle is farther than a threshold.
            If the ego vehicle is too far (more than 50m), reduce the distance by moving the leading vehicle.
            """
            ego_location = self.ego_vehicles[0].get_location()
            leading_vehicle_location = self.other_actors[0].get_location()
            distance = ego_location.distance(leading_vehicle_location)

            # Threshold distance (meters)
            stop_threshold = 10.0  # When to stop the leading vehicle
            move_threshold = 50.0  # When to move the leading vehicle closer to ego vehicle

            if distance < move_threshold:
                # Move the leading vehicle closer by applying throttle
                self.other_actors[0].set_autopilot(True)
                self.other_actors[0].apply_control(carla.VehicleControl(throttle=0.5, brake=0.0))  # Apply throttle to move closer
                print(f"Leading vehicle is moving closer. Distance: {distance:.2f}")
                return py_trees.common.Status.RUNNING  # Keep moving until the distance is within range

            elif distance > move_threshold:
                # Stop the leading vehicle if the ego vehicle is close enough
                self.other_actors[0].apply_control(carla.VehicleControl(throttle=0.0, brake=5.0))  # Stop the vehicle
                self.other_actors[0].set_autopilot(False)
                print(f"Leading vehicle stopped. Distance: {distance:.2f}")
                return py_trees.common.Status.RUNNING  # Keep the vehicle stopped until ego car gets closer

            else:
                # Resume autopilot when within the desired range
                self.other_actors[0].set_autopilot(True)
                print(f"Leading vehicle resumes. Distance: {distance:.2f}")
                return py_trees.common.Status.SUCCESS

        # Wrap the logic in a py_trees behavior
        stop_behavior = py_trees.behaviours.Running(name="Check Proximity and Control Leading Vehicle")
        stop_behavior.update = stop_leading_vehicle_if_far

        # Add the stop behavior to the sequence
        sequence.add_child(stop_behavior)

        # Add a behavior to drive the ego vehicle
        def ego_drive():
            """
            Drive the ego vehicle.
            """
            ego_velocity = self.ego_vehicles[0].get_velocity()
            if ego_velocity.length() > 0.1:
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.SUCCESS

        # Wrap ego driving as a behavior
        ego_drive_behavior = py_trees.behaviours.Running(name="Drive Ego")
        ego_drive_behavior.update = ego_drive

        # Add the drive behavior to the sequence
        sequence.add_child(ego_drive_behavior)

        return sequence

    def _create_test_criteria(self):
        # Add criteria if needed
        pass

    def __del__(self):
        self.LOG_insert("file.log", "Finishing scenario", logging.INFO)
        self.remove_all_actors()

    def _get_waypoint_in_distance(self, waypoint, distance):
        """
        Find a waypoint at a specified distance from the given waypoint.
        """
        next_waypoint = waypoint
        traveled_distance = 0.0

        while traveled_distance < distance:
            next_waypoints = next_waypoint.next(2.0)  # Distance increment
            if not next_waypoints:
                break
            next_waypoint = next_waypoints[0]
            traveled_distance += 2.0

        return next_waypoint, traveled_distance
