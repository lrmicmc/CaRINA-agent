#!/bin/bash
#ROS_PYTHON_PATH=/opt/ros/melodic/lib/python2.7/dist-packages

# CARLA BASH CONFIGURATION
export CARLA_VERSION="Leaderboard_20_14"


  
export CARLA_ROS_BRIDGE_ROOT=/home/luis/carla2/ros-bridge
export CARLA_ROOT=/home/luis/carla2/CARLA_$CARLA_VERSION/
export CARLA_SERVER=/home/luis/carla2/CARLA_$CARLA_VERSION/CarlaUE4.sh
export LEADERBOARD_ROOT=/home/luis/carla2/leaderboard
export TRACK_ROOT=/home/luis/carla/TRACK
export TEAM_CONFIG=""
export SCENARIO_RUNNER_ROOT=/home/luis/carla2/scenario_runner

export MMSEGMENTATION=/home/luis/mmsegmentation
export MMDETECTION=/home/luis/mmdetection
export MMDETECTION3D=/home/luis/mmdetection3d
                     
export TEAM_CODE_ROOT=/home/luis/carla/TRACK/team_code

if [ -z "$CARLA_ROOT" ]
then
    echo "Error $CARLA_ROOT is empty. Set \$CARLA_ROOT as an environment variable first."
    #exit 1
fi

if [ -z "$SCENARIO_RUNNER_ROOT" ]
then echo "Error $SCENARIO_RUNNER_ROOT is empty. Set \$SCENARIO_RUNNER_ROOT as an environment variable first."
    #exit 1
fi

if [ -z "$LEADERBOARD_ROOT" ]
then echo "Error $LEADERBOARD_ROOT is empty. Set \$LEADERBOARD_ROOT as an environment variable first."
    #exit 1
fi

if [ -z "$TEAM_CODE_ROOT" ]
then echo "Error $TEAM_CODE_ROOT is empty. Set \$TEAM_CODE_ROOT as an environment variable first."
    #exit 1
fi

if [ -z "$MMDETECTION" ]
then echo "Error $MMDETECTION is empty. Set \$MMDETECTION as an environment variable first."
    #exit 1
fi

if [ -z "$MMSEGMENTATION" ]
then echo "Error $MMSEGMENTATION is empty. Set \$MMSEGMENTATION as an environment variable first."
    #exit 1
fi

if [ -z "$MMDETECTION3D" ]
then echo "Error $MMDETECTION3D is empty. Set \$MMDETECTION3D as an environment variable first."
    #exit 1
fi

if [ -z "$CARLA_ROS_BRIDGE_ROOT" ]
then echo "Error $CARLA_ROS_BRIDGE_ROOT is empty. Set \$CARLA_ROS_BRIDGE_ROOT as an environment variable first."
    #exit 1
fi

rm -fr .tmp
mkdir -p .tmp

cp -fr ${CARLA_ROOT}/PythonAPI  .tmp
#mv .tmp/PythonAPI/carla/dist/carla*-py2*.egg .tmp/PythonAPI/carla/dist/carla-leaderboard-py2.7.egg
mv .tmp/PythonAPI/carla/dist/carla*-py3*.egg .tmp/PythonAPI/carla/dist/carla-leaderboard-py3x.egg

cp -fr ${CARLA_ROS_BRIDGE_ROOT}/ .tmp/carla_ros_bridge
cp -fr ${MMDETECTION} .tmp
rm -fr .tmp/mmdetection/.git
rm -fr .tmp/mmdetection/work_dirs
cp -fr ${MMSEGMENTATION} .tmp
rm -fr .tmp/mmsegmentation/.git
rm -fr .tmp/mmsegmentation/work_dirs
cp -fr ${MMDETECTION3D} .tmp
rm -fr .tmp/mmdetection3d/.git
rm -fr .tmp/mmdetection3d/work_dirs

cp -fr ${SCENARIO_RUNNER_ROOT}/ .tmp
rm -fr .tmp/scenario_runner/.git
cp -fr ${LEADERBOARD_ROOT}/ .tmp
rm -fr .tmp/leaderboard/.git
cp -fr ${TEAM_CODE_ROOT}/ .tmp/team_code
cp ${TEAM_CODE_ROOT}/agent_entrypoint.sh .tmp/agent_entrypoint.sh
cp ${TEAM_CODE_ROOT}/bashrc/.bashrc .tmp/.bashrc


# build docker image
#docker build --force-rm -t leaderboard-sensors -f ${LEADERBOARD_ROOT}/scripts/Dockerfile_leaderboard_melodic.ros .
#docker build --force-rm -t leaderboard-sensors-l2 -f ${TEAM_CODE_ROOT}/make_docker_lrm/Dockerfile_leaderboard.ros .
docker build --force-rm -t leaderboard-map-l2 -f ${TEAM_CODE_ROOT}/make_docker_lrm/Dockerfile_leaderboard.ros .
rm -fr .tmp
