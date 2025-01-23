import subprocess
import random

commands = {
    "Route: 10, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 10 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/10/0 --record study/000/10/0",
    "Route: 10, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 10 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/10/1 --record study/000/10/1",
    "Route: 10, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 10 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/10/2 --record study/000/10/2",
    "Route: 10, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 10 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/10/3 --record study/000/10/3",
    "Route: 11, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 11 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/11/0 --record study/000/11/0",
    "Route: 11, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 11 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/11/1 --record study/000/11/1",
    "Route: 11, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 11 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/11/2 --record study/000/11/2",
    "Route: 11, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 11 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/11/3 --record study/000/11/3",
    "Route: 20, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 20 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/20/0 --record study/000/20/0",
    "Route: 20, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 20 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/20/1 --record study/000/20/1",
    "Route: 20, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 20 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/20/2 --record study/000/20/2",
    "Route: 20, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 20 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/20/3 --record study/000/20/3",
    "Route: 21, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 21 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/21/0 --record study/000/21/0",
    "Route: 21, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 21 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/21/1 --record study/000/21/1",
    "Route: 21, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 21 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/21/2 --record study/000/21/2",
    "Route: 21, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 21 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/21/3 --record study/000/21/3",
    "Route: 30, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 30 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/30/0 --record study/000/30/0",
    "Route: 30, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 30 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/30/1 --record study/000/30/1",
    "Route: 30, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 30 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/30/2 --record study/000/30/2",
    "Route: 30, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 30 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/30/3 --record study/000/30/3",
    "Route: 31, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 31 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/31/0 --record study/000/31/0",
    "Route: 31, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 31 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/31/1 --record study/000/31/1",
    "Route: 31, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 31 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/31/2 --record study/000/31/2",
    "Route: 31, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 31 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/31/3 --record study/000/31/3",
    "Route: 40, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 40 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/40/0 --record study/000/40/0",
    "Route: 40, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 40 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/40/1 --record study/000/40/1",
    "Route: 40, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 40 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/40/2 --record study/000/40/2",
    "Route: 40, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 40 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/40/3 --record study/000/40/3",
    "Route: 41, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 41 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/41/0 --record study/000/41/0",
    "Route: 41, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 41 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/41/1 --record study/000/41/1",
    "Route: 41, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 41 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/41/2 --record study/000/41/2",
    "Route: 41, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 41 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/41/3 --record study/000/41/3",
    "Route: 50, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 50 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/50/0 --record study/000/50/0",
    "Route: 50, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 50 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/50/1 --record study/000/50/1",
    "Route: 50, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 50 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/50/2 --record study/000/50/2",
    "Route: 50, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 50 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/50/3 --record study/000/50/3",
    "Route: 51, Level: (0.0, 0.0)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 51 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/51/0 --record study/000/51/0",
    "Route: 51, Level: (0.02, 0.03)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 51 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/51/1 --record study/000/51/1",
    "Route: 51, Level: (0.05, 0.075)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 51 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/51/2 --record study/000/51/2",
    "Route: 51, Level: (0.1, 0.15)" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 51 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study/000/51/3 --record study/000/51/3"
}
# Random order instead of sorted
#shuffled_commands = list(commands.items())
#random.shuffle(shuffled_commands)

for route, command in commands.items():
    print(f"Upcoming: {route}")
    print(f"Enter Level and press enter to start route")
    input()

    print(f"Running: {command}")
    process = subprocess.run(command, shell=True)
    if process.returncode != 0:
        print(f"Route '{route}' failed with return code {process.returncode}")
    print("\nRoute completed.")