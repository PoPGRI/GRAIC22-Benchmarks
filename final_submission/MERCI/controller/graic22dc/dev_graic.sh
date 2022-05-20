#!/usr/bin/env bash

# WARNING: this kills tmux and docker resources and requires root access
# GRAIC TMux Session Script
#
# First, make sure to manage docker as non root user: https://docs.docker.com/engine/install/linux-postinstall/
# and make sure the socker can be used as non-root:
#     sudo chmod 666 /var/run/docker.sock
#
# Usage: in a directory with user_controller_file.py, run
#     sudo path/to/dev_graic.sh <graic22dc-folder>
#
# <graic22dc-folder> is the folder that contains the graic22dc module.
# This will be mounted on docker and executed.
# By default, this will be the parent of current directory
arg1=${1:-$PWD/..}

# t1_triple, t2_triple, t3, t4, track5
track_type_arg=${2}

read -p "Kill Conflicting TMux Sessions and Docker Images?(Y/N): " input
if [[ $input == [yY] ]]; then
        docker kill graic_con
        tmux kill-session -t graic-dev
fi

tmux new-session -d -s graic-dev
echo "Starting Carla..."
tmux send "docker run --name graic_con --privileged --rm --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --net=host -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ${arg1}:/home/carla/graic22dc:rw sundw2014/graic /bin/bash" ENTER;
sleep 2;
tmux send '~/workspace/carla-simulator/CarlaUE4.sh -opengl' ENTER;

echo "Starting the Carla-Ros Bridge..."
sleep 10;
tmux split-window -h;
tmux select-pane -t 0;
tmux split-window -v;
tmux select-pane -t 1;
tmux send 'docker exec -it graic_con /bin/bash' ENTER;             
tmux send '~/scripts/update.sh' ENTER;
tmux send '. ~/workspace/graic-workspace/devel/setup.bash' ENTER;
tmux send "roslaunch graic_core graic_single.launch synchronous_mode_wait_for_vehicle_control_command:=True model_type:=model_free vis2D:=True track:=${track_type_arg}" ENTER;

echo "Preparing Controller Environment..."
tmux select-pane -t 2;
tmux send 'docker exec -it graic_con /bin/bash' ENTER;            
sleep 2;
tmux send 'cd /home/carla/graic22dc' ENTER;
tmux send 'cp ~/workspace/graic-workspace/src/graic_core/src/agent_wrapper.py .' ENTER;
tmux send '. ~/workspace/graic-workspace/devel/setup.bash' ENTER;
tmux send 'export PYTHONPATH+=":/home/carla/"' ENTER;
tmux send 'pip install joblib sklearn ipython' ENTER;
tmux send 'python3 agent_wrapper.py ego_vehicle'
tmux a;
