SETUP="source /home/carla/workspace/graic-workspace/devel/setup.bash"
ROSLAUNCH="roslaunch graic_core graic_single.launch synchronous_mode_wait_for_vehicle_control_command:=True model_type:=model_free vis2D:=True"
#docker exec -it graic_con /bin/bash -c "~/scripts/update.sh"#;$SETUP;$ROSLAUNCH"
docker exec -it graic_con /bin/bash -c "echo Type the echoed commands && echo source /home/carla/workspace/graic-workspace/devel/setup.bash && echo roslaunch graic_core graic_single.launch synchronous_mode_wait_for_vehicle_control_command:=True model_type:=model_free vis2D:=True;bash"
