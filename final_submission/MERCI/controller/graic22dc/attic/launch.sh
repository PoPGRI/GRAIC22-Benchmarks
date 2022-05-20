#!/usr/bin/env bash
set -e
TERM="gnome-terminal"
DOCKER="$TERM -- bash -c ./scripts/docker.sh;bash"
CARLA="$TERM -- bash -c ./scripts/carla.sh"
ROS="$TERM -- bash -c ./scripts/ros.sh;bash"
CONTROL="$TERM -- bash -c ./scripts/run_controller.sh;bash"

# The below ordering is important
echo "starting docker..."
$DOCKER
sleep 5
echo "starting CARLA..."
$CARLA
sleep 5
$ROS
read -p "Start ROS and press any key to continue..."
echo "running controller..."
$CONTROL
