#!/bin/bash

_GREEN='\e[32m'
_NORMAL='\e[0m'
_BOLD='\e[33m'
_RED='\e[31m'
_YELLOW="\033[0;36m"
_BLUE="\033[0;34m"
_PURPLE="\033[0;35m"
_RED_ALT="\033[0;31m"

echo -e "${_BOLD}--------------------------${_NORMAL}"
echo -e "${_YELLOW} agilex ${_NORMAL}"
echo -e "${_YELLOW} select limo version ${_NORMAL}"
echo -e "${_BOLD}--------------------------${_NORMAL}"
echo -e "${_GREEN} 1. limo ${_NORMAL}"
echo -e "${_BLUE} 2. limo pro ${_NORMAL}"
echo -e "${_YELLOW} 3. limo ros2 ${_NORMAL}"

echo -n "Your choice (1-3): "

read CHOOSE


function limo() {
    roslaunch astra_camera dabai_u3.launch  || { echo -e "${_RED}Failed to launch ros_astra_camera${_NORMAL}"; exit 1; }
    rqt_image_view
}

function limo_pro() {
    sudo rm -r ../agilex_ws/src/ros_astra_camera/ || { echo -e "${_RED}Failed to remove old ros_astra_camera${_NORMAL}"; exit 1; }
    
}

function limo_ros2() {
    FILENAME=~/agilex_ws/src/ros2_astra_camera/astra_camera/params/dabai_params.yaml
}

case "${CHOOSE}" in
    1)
        limo
        ;;
    2)
        limo_pro
        ;;
    3)
        limo_ros2
        ;;
    *)
        echo -e "${_RED}Invalid choice. Please choose 1, 2, or 3.${_NORMAL}"
        exit 1
        ;;
esac

echo -e "${_GREEN}Complete operation!${_NORMAL}"
