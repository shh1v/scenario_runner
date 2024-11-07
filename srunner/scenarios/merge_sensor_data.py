import pandas as pd

# Load both CSV files
sensor_data = pd.read_csv("D:\CarlaDReyeVR\scenario_runner\sensor_data.csv")
hmd_data = pd.read_csv("D:\CarlaDReyeVR\carla\Unreal\CarlaUE4\CustomHMDData.csv")

# Ensure both have a Unix Timestamp column for merging
sensor_data["Unix Timestamp"] = pd.to_numeric(sensor_data["Unix Timestamp"], errors='coerce')
hmd_data["Unix Timestamp"] = pd.to_numeric(hmd_data["Unix Timestamp"], errors='coerce')

# Merge based on Unix Timestamp with an outer join to keep all data
merged_data = pd.merge_asof(
    sensor_data.sort_values("Unix Timestamp"),
    hmd_data.sort_values("Unix Timestamp"),
    on="Unix Timestamp",
    direction="nearest",  
    tolerance=0.1  
)

# Write the merged data to a new CSV file
merged_data.to_csv("merged_sensor_data.csv", index=False)

print("Merge complete. Merged data saved to merged_sensor_data.csv.")
