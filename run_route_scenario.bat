:: run this from scenario runner root
:: change to where you installed CARLA
SET CARLA_ROOT=C:\AwarenessManager\carla.awareness_harp
:: change to port that CARLA is running on
SET PORT=2000  
:: export ROUTE=leaderboard\data\route11.xml   # change to desired route
@REM SET ROUTE_FILE=srunner\data\routes_custom.xml
@REM SET SCENARIO_FILE=srunner\data\town03_scenarios.json
@REM @REM SET SCENARIO_FILE=leaderboard\data\periph_study_scenarios.json
@REM SET ROUTE_NUM=32


@REM SET ROUTE_FILE=srunner\data\routes_gus.xml
@REM SET SCENARIO_FILE=srunner\data\gus-thesis\scenarios_gus.json
@REM SET ROUTE_NUM=111
@REM SET ROUTE_NUM=911501

SET ROUTE_FILE=srunner\data\routes_custom.xml
SET SCENARIO_FILE=srunner\data\town05_scenarios.json
SET ROUTE_NUM=54

::clear PYTHONPATH so this doesn't accumulate
SET PYTHONPATH=
SET PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\carla
SET PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\carla\dist\carla-0.9.13-py3.7-win-amd64.egg
:: for DReyeVR_utils
SET PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\examples 


python scenario_runner.py --route %ROUTE_FILE% %SCENARIO_FILE% %ROUTE_NUM% --debug --sync