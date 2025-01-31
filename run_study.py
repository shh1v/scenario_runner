import subprocess
#import random

commands = {
    "10 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 10 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/10/0 --record study/000/10/0",
    "10 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 10 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/10/1 --record study/000/10/1",
    "10 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 10 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/10/2 --record study/000/10/2",
    "10 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 10 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/10/3 --record study/000/10/3",
    "11 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 11 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/11/0 --record study/000/11/0",
    "11 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 11 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/11/1 --record study/000/11/1",
    "11 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 11 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/11/2 --record study/000/11/2",
    "11 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 11 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/11/3 --record study/000/11/3",
    "20 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 20 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/20/0 --record study/000/20/0",
    "20 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 20 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/20/1 --record study/000/20/1",
    "20 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 20 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/20/2 --record study/000/20/2",
    "20 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 20 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/20/3 --record study/000/20/3",
    "21 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 21 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/21/0 --record study/000/21/0",
    "21 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 21 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/21/1 --record study/000/21/1",
    "21 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 21 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/21/2 --record study/000/21/2",
    "21 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/2_stop_sign.json 21 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/21/3 --record study/000/21/3",
    "30 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 30 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/30/0 --record study/000/30/0",
    "30 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 30 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/30/1 --record study/000/30/1",
    "30 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 30 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/30/2 --record study/000/30/2",
    "30 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 30 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/30/3 --record study/000/30/3",
    "31 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 31 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/31/0 --record study/000/31/0",
    "31 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 31 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/31/1 --record study/000/31/1",
    "31 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 31 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/31/2 --record study/000/31/2",
    "31 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/3_bicycle.json 31 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/31/3 --record study/000/31/3",
    "40 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 40 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/40/0 --record study/000/40/0",
    "40 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 40 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/40/1 --record study/000/40/1",
    "40 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 40 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/40/2 --record study/000/40/2",
    "40 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 40 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/40/3 --record study/000/40/3",
    "41 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 41 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/41/0 --record study/000/41/0",
    "41 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 41 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/41/1 --record study/000/41/1",
    "41 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 41 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/41/2 --record study/000/41/2",
    "41 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/4_pedestrian.json 41 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/41/3 --record study/000/41/3",
    "50 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 50 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/50/0 --record study/000/50/0",
    "50 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 50 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/50/1 --record study/000/50/1",
    "50 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 50 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/50/2 --record study/000/50/2",
    "50 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 50 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/50/3 --record study/000/50/3",
    "51 0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 51 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/51/0 --record study/000/51/0",
    "51 1" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 51 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/51/1 --record study/000/51/1",
    "51 2" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 51 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/51/2 --record study/000/51/2",
    "51 3" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/5_construction.json 51 --agent srunner/autoagents/dummy_agent.py --timeout 50 --sync --output --file --outputDir study/000/51/3 --record study/000/51/3"
}

# Random order instead of sorted
#shuffled_commands = list(commands.items())
#random.shuffle(shuffled_commands)

# for route, command in commands.items():
#     print(f"Upcoming: {route}")
#     print(f"Enter Level and press enter to start route")
#     input()

#     print(f"Running: {command}")
#     process = subprocess.run(command, shell=True)
#     if process.returncode != 0:
#         print(f"Route '{route}' failed with return code {process.returncode}")
#     print("\nRoute completed.")

while True:
    route_input = input("Enter route and level (e.g., '10 0') or 'q' to quit: ").strip()
    if route_input.lower() == "q":
        print("Exiting...")
        break
    if route_input in commands:
        confirm = input(f"You selected '{route_input}'. Press enter to start or type 'cancel' to abort: ").strip()
        if confirm.lower() != "cancel":
            print(f"Running: {commands[route_input]}")
            process = subprocess.run(commands[route_input], shell=True)
            if process.returncode != 0:
                print(f"Route '{route_input}' failed with return code {process.returncode}")
            print("\nRoute completed.")
    else:
        print("Invalid route. Please try again.")