#!/bin/bash

# Absolute path to your Python script
python_script="/home/otonom/fgm_ws/src/wheelchair_simulations/src/automatesimulations.py"

# Function to run the Python script in the background
run_python_script() {
  python3 $python_script
}

# Run the Python script for the first time
run_python_script

# Loop to continuously restart the Python script
while true; do
  # Wait for the Python script to finish
  wait

  # Wait for 10 seconds before restarting the Python script
  sleep 10

  # Run the Python script in the background
  run_python_script
done
