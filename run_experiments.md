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
Open 3 terminals in the docker using the below commands:

    docker exec -it graic_test bash

In Terminal 1:

    ./runCarla.sh 2000

In Terminal 2:

    ./launch.sh 2000 "model_free" "t1_triple" 

In Terminal 3:

    ./race.sh graic2021@gmail.com ${folder_name}
    
When Terimal 3 has finishde the above process, generate the video:

    ./jpg_2_mp4.sh
Notes:
1. Replace folder_name with the the team name that you want to run experiments on(E.g. `EMI`).
2. The testing docker currently stores the latest submission of all GRAIC 2022 teams. If you want to run previous versions of the submitted code, please use `docker cp` command to replace the existsing folder with your desired one. 
3. There are currently 5 tracks provided by GRAIC 2022. `t1_triple`, `t2_triple`, `t3`, `t4`, `track5`. Replace the t1_triple above in you want run experiments on other tracks

## Output (stored in /home/carla)
- output_ego_vehicle.mp4
    - Note that this docker image does not come with images, the mp4 is to provide users with visual feedback.
- score_ego_vehicle.txt
    - Details about time to finish and collision
- record.txt
    - Every event at the race(Position, velocity, obstacle information...)


## Re-run the Experiements
Due to some unstability from the Carla Simulator, killing and restarting the process is somtimes needed; otherwise the simulator might fail.

    pkill -9 -f python3
    pkill -9 -f ros 
    pkill -9 -f Carla  
