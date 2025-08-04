#!/bin/bash
poczatkowa_lokalizacja="$(pwd)"
aktualna_data=$(date +"%Y-%m-%d_%H-%M-%S")

cd ~/ros2_ws/drzewka_tf || exit 1
mkdir -p "$aktualna_data"
cd "$aktualna_data" || exit 1

ros2 run tf2_tools view_frames

cd "$poczatkowa_lokalizacja" || exit 1