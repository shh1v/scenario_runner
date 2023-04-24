:: run this from scenario runner root
:: change to where you installed CARLA
SET CARLA_ROOT=C:\AwarenessManager\carla.awareness_harp
:: change to port that CARLA is running on
SET PORT=2000  
:: export ROUTE=leaderboard\data\route11.xml   # change to desired route
SET ROUTE_FILE=srunner\data\longest6\longest6.xml
SET SCENARIO_FILE=srunner\data\longest6\eval_scenarios.json
@REM SET SCENARIO_FILE=leaderboard\data\periph_study_scenarios.json

::clear PYTHONPATH so this doesn't accumulate
SET PYTHONPATH=
SET PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\carla
SET PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\carla\dist\carla-0.9.13-py3.7-win-amd64.egg
:: for DReyeVR_utils
SET PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\examples 

SET ROUTE_NUM=20

python scenario_runner.py --route %ROUTE_FILE% %SCENARIO_FILE% %ROUTE_NUM% --debug