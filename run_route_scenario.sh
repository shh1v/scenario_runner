# run this from scenario runner root
export CARLA_ROOT=/mnt/c/AwarenessManager/carla.awareness_harp  # change to where you installed CARLA
export PORT=2000  # change to port that CARLA is running on
export ROUTE=leaderboard/data/route11.xml   # change to desired route
SCENARIO_FILE=leaderboard/data/dreyevr/periph_study_scenarios.json

export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.6-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/examples # for DReyeVR_utils


if [ -d "$TEAM_CONFIG" ]; then
    CHECKPOINT_ENDPOINT="$TEAM_CONFIG/$(basename $ROUTES .xml).txt"
else
    CHECKPOINT_ENDPOINT="$(dirname $TEAM_CONFIG)/$(basename $ROUTES .xml).txt"
fi

python3 scenario_runner.py \
--route=${TEAM_AGENT} \
--scenarios=${SCENARIO_FILE}  \
--port=${PORT}

export PYTHONPATH="\mnt\c\AwarenessManager\carla.awareness_harp\PythonAPI\carla\"
