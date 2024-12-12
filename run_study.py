import subprocess

commands = {
    "Route: 10, Severity: 0.0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 10 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study_data/ --record study_data/",
    "Route: 11, Severity: 0.0" : "python scenario_runner.py --route srunner/data/routes_study.xml srunner/data/1_traffic_lights_l.json 11 --agent srunner/autoagents/dummy_agent.py --timeout 20 --sync --output --file --outputDir study_data/ --record study_data/"
}

for route, command in commands.items():
    print(f"Running: {route}")
    print(f"Running: {command}")
    process = subprocess.run(command, shell=True)
    if process.returncode != 0:
        print(f"Route '{route}' failed with return code {process.returncode}")
    print("\nRoute completed.")
    print(f"Next route: {route}")
    print(f"Enter to start next route")
    input()