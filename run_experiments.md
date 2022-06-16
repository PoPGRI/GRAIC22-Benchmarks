# Running GRAIC2022 With Submitted Controllers

---

Tested working with the below Configurations
- GPU: Quadro P5000
- Docker version 20.10.7
- Sudo access on your machine
- Ubuntu 18.04 and 20.04

--- 

## Step 1: Pull the GRAIC testing docker image
    docker pull sundw2014/graic-testing-graic
The docker image is based on Ubuntu 20.04 and has pre-installed 0.9.11 Carla Simulator and 0.9.11 Scenario Runner.

## Step 2: Launch the docker 
    sudo chmod 777 /var/run/docker.sock
    docker run -dit --name graic_test --privileged --rm --gpus all --env NVIDIA_DISABLE_REQUIRE=1 sundw2014/graic-testing-graic

## Step 3: Launch Testing Script
    docker exec graic_test ./runCarla.sh 2000 &
    docker exec graic_test sleep 15
    docker exec graic_test ./launch.sh 2000 "model_free" "t1_triple" 
    docker exec graic_test sleep 15
    docker exec graic_test ./race.sh graic2021@gmail.com ${folder_name}
    docker exec graic_test ./jpg_2_mp4.sh
Notes:
1. You could either put above commands in one script and then execute, or open a new terminal for each command.
2. Replace folder_name with the the team name that you want to run experiments on(E.g. `EMI`).
3. The testing docker currently stores the latest submission of all GRAIC 2022 teams. If you want to run previous versions of code, please use `docker cp` command to replace the existsing folder with your desired one. 
4. There are currently 5 tracks provided by GRAIC 2022. `t1_triple`, `t2_triple`, `t3`, `t4`, `track5`. Replace the t1_triple above in you want run experiments on other tracks

## Output (stored in /home/carla)
- output_ego_vehicle.mp4
    - Note that this docker image does not come with images, the mp4 is to provide users with visual feedback.
- score_ego_vehicle.txt
    - Details about time to finish and collision
- record.txt
    - Every event at the race(Position, velocity, obstacle information...)


## Re-run the Experiements
Due to some unstability from the Carla Simulator, killing and restarting the process is somtimes needed; otherwise the simulator might fail.

    docker exec graic_test pkill python3 || exit 0
    docker exec graic_test pkill ros || exit 0
    docker exec graic_test pkill Carla || exit 
