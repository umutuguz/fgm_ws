#!/bin/bash

# Number of worlds to generate
NUM_WORLDS=500

# Loop to generate worlds
for ((i=1; i<=$NUM_WORLDS; i++))
do
    # Generate a unique world name
    WORLD_NAME="world_$i"

    # Launch the Gazebo world and spawning objects script
    roslaunch wheelchair_simulations sdf_launcher.launch &
    
    # Wait for Gazebo to open
    sleep 8

    # Run the spawning objects script
    rosrun wheelchair_simulations spawn_boxes.py

    # Save the modified world with a new name
    gz model --save -m wheelchair_simulations -w default -o output/$WORLD_NAME.world
    
    sleep 6

    # Close Gazebo
    pkill -9 -f gzserver
    pkill -9 -f gzclient
done

