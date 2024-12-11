from DReyeVR_utils import DReyeVRSensor
from datetime import datetime
import csv
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
        with open("study_data/sensor_data.csv", "a+", newline='') as f:
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