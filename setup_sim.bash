#! /bin/bash


source /opt/ros/melodic/setup.bash
catkin build
source devel/setup.bash

#Check if the PX4 Firmware Folder Exists in user's home directory
if [ -d /home/$USER/Firmware ]
then
    echo "~/Firmware exists on your filesystem."
else
    echo "Do you want to install PX4 SITL?(Y/N)" 
    read uservar
    if [ $uservar = "Y" ]
    then
        mkdir - p /home/$USER/big_test
        cd /home/$USER/big_test
        git clone https://github.com/PX4/Firmware.git
        cd Firmware
        git submodule update --init --recursive
        DONT_RUN=1 make px4_sitl_default gazebo
    else
        exit
    fi
fi

echo "Do you want to launch the PX4 simulation:(Y/N)"
read uservar
if [ $uservar = "Y" ]
    then
        cd /home/$USER/Firmware
        DONT_RUN=1 make px4_sitl_default gazebo
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware/Tools/sitl_gazebo
        echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH
        roslaunch simulation px4_sim.launch
    else
        exit
fi