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

function create_udev() {
    echo ""
    echo "This script copies a udev rule to /etc to facilitate bringing"
    echo "up the astra usb connection as /dev/astra*"
    echo ""

    sudo cp 56-orbbec-usb.rules /etc/udev/rules.d || { echo -e "${_RED}Failed to copy udev rule${_NORMAL}"; exit 1; }

    echo ""
    echo "Restarting udev"
    echo ""
    sleep 0.2
    sudo service udev reload || { echo -e "${_RED}Failed to reload udev${_NORMAL}"; exit 1; }
    sleep 0.2
    sudo service udev restart || { echo -e "${_RED}Failed to restart udev${_NORMAL}"; exit 1; }
}

function limo() {
    create_udev &&
    cd libuvc/build &&
    sudo rm -r ./* &&
    cmake .. &&
    make &&
    sudo make install &&
    cd ../.. &&

    sudo rm -r ~/agilex_ws/src/ros_astra_camera/ || { echo -e "${_RED}Failed to remove old ros_astra_camera${_NORMAL}"; exit 1; }
    cp -r ros_astra_camera ~/agilex_ws/src || { echo -e "${_RED}Failed to copy new ros_astra_camera${_NORMAL}"; exit 1; }
    cd ~/agilex_ws || { echo -e "${_RED}Failed to change directory to ~/agilex${_NORMAL}"; exit 1; }
    catkin_make -DCATKIN_WHITELIST_PACKAGES=astra_camera || { echo -e "${_RED}Failed to run catkin_make${_NORMAL}"; exit 1; }
}

function limo_pro() {
    sudo rm -r ~/agilex_ws/src/ros_astra_camera/ || { echo -e "${_RED}Failed to remove old ros_astra_camera${_NORMAL}"; exit 1; }
    cp -r ros_astra_camera ~/agilex_ws/src || { echo -e "${_RED}Failed to copy new ros_astra_camera${_NORMAL}"; exit 1; }
    cd ~/agilex_ws || { echo -e "${_RED}Failed to change directory to ~/agilex${_NORMAL}"; exit 1; }
    catkin_make -DCATKIN_WHITELIST_PACKAGES=astra_camera || { echo -e "${_RED}Failed to run catkin_make${_NORMAL}"; exit 1; }


    FILENAME=~/limo_ros2_ws/src/ros2_astra_camera/astra_camera/params/dabai_params.yaml
    # 使用sed命令进行替换
    sed -i 's/0x050e/0x0557/g' "$FILENAME"
    # 输出替换后的结果
    echo "Replaced 0x050e with 0x0557 in $FILENAME"

    FILENAME=~/limo_ros2_ws/install/astra_camera/share/astra_camera/params/dabai_params.yaml
    # 使用sed命令进行替换
    sed -i 's/0x050e/0x0557/g' "$FILENAME"
    # 输出替换后的结果
    echo "Replaced 0x050e with 0x0557 in $FILENAME"
}

function limo_ros2() {
    FILENAME=~/agilex_ws/src/ros2_astra_camera/astra_camera/params/dabai_params.yaml
    FILENAME2=~/agilex_ws/install/astra_camera/share/astra_camera/params/dabai_params.yaml
    # 使用sed命令进行替换
    sudo sed -i 's/0x050e/0x0557/g' "$FILENAME"
    sudo sed -i 's/0x050e/0x0557/g' "$FILENAME2"

    # 输出替换后的结果
    echo "Replaced 0x050e with 0x0557 in $FILENAME"
    echo "Replaced 0x050e with 0x0557 in $FILENAME2"
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
