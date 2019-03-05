#!/usr/bin/zsh

# Setup file
SETUP_FILE=./devel/setup.zsh
source $SETUP_FILE 
echo "$SETUP_FILE loaded."

# Turtlebot3 model
export TURTLEBOT3_MODEL=burger
echo "Exported: TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"

# Gazebo description file
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
echo "Exported: HUSKY_GAZEBO_DESCRIPTION=$HUSKY_GAZEBO_DESCRIPTION"

#  Environment variables for correct visualization
export QT_AUTO_SCREEN_SCALE_FACTOR=
export QT_SCREEN_SCALE_FACTORS=
