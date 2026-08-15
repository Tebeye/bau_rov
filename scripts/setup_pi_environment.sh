#!/usr/bin/env bash
# ==============================================================================
# T-1.1 & T-1.3: Raspberry Pi 4/5 Ubuntu Server 22.04 LTS Setup & MAVROS
# ==============================================================================
set -e

echo "[+] Updating system packages..."
sudo apt update && sudo apt upgrade -y

echo "[+] Installing ROS 2 Humble dependencies and build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-opencv \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-nav-msgs \
    ros-humble-mavros \
    ros-humble-mavros-extras



echo "[+] Configuring serial port permissions..."
sudo usermod -a -G dialout $USER

echo "[+] Setup completed successfully!"
