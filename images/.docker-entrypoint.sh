#!/bin/bash

# Since this entrypoint is run as root, the environment variable passed in run or compose will not be available to the container user, that's why we write them out manually here

# Import docker-compose env into user domain
echo "ROS_MASTER=${ROS_MASTER}" >> $HOME/.env

# Source the env and set ROS_MASTER in user .bashrc accordingly
# -------------------------------------------------------------------
echo 'source /.env.sh' >> $HOME/.bashrc # This will source the env file with every new terminal instance
echo 'export ROS_MASTER_URI=http://${ROS_MASTER}:11311' >> $HOME/.bashrc

exec "$@"