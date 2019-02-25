#!/usr/bin/zsh


# Define GIT repositories to clone
#repos=('https://github.com/husky/husky.git' \
       #'https://github.com/ros-drivers/um6.git' \
       #'https://github.com/ros-teleop/teleop_twist_joy.git' \
       #'https://github.com/ros-perception/openslam_gmapping' \
       #'https://github.com/ros-perception/slam_gmapping' \
       #'https://github.com/clearpathrobotics/cpr_multimaster_tools.git' \
       #'https://github.com/ros-drivers/nmea_comms.git' \
       #'https://github.com/ros-perception/imu_pipeline.git' \
       #'https://github.com/ros-drivers/microstrain_3dmgx2_imu.git' \
       #'https://github.com/paulbovbel/frontier_exploration.git' \
       #'https://github.com/ros-teleop/teleop_twist_keyboard',
        #)
repos=( \
    'https://github.com/husky/husky.git' \
    'https://github.com/davidvg/LMS1xx.git' \
    'https://github.com/ros-visualization/interactive_marker_twist_server.git' \
    )

ROSDEP_INSTALL=false

echo "Inititating workspace..."
# Make SRC_DIR and enter it
mkdir -p ./src
cd ./src

# Clone the repositories defined in repos
echo "\nClone GIT repositories..."
for repo in $repos
do
    args=""
    repo_name=$(echo $repo | cut -d "/" -f 5 | cut -d "." -f 1)
    if [ ! -d $repo_name ]
    then
        git clone $args $repo
    else
        echo "Repository $repo already exists."
    fi
done
echo "\nDone."

cd ..


# Install packages using rosdep
if [ $ROSDEP_INSTALL = true ]
then
    echo "\nInstalling packages..."
    rosdep install --from-paths . -i
fi

echo "\nCompiling code..."
catkin build

source ./load_env.sh
