FROM osrf/ros:humble-desktop-jammy

# 環境変数を設定
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# パッケージリストを更新し、基本的なツールをインストール
RUN apt-get update && apt-get install -y \
    sudo \
    lsb-release \
    gnupg \
    curl \
    wget \
    vim \
    git \
    python3-pip \
    # debugのためのツール
    gdb \
    build-essential

# Gazeboのリポジトリキーとリポジトリを追加
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update &&  apt-get install -y  gz-harmonic

# MoveIt2のインストール
RUN apt-get update && apt-get install -y \
    ros-humble-moveit

# ROS2 Controlのインストール
RUN apt-get update && apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# lbr_fri_ros2_stack
RUN apt-get update && apt-get install -y \
    ros-humble-pilz-industrial-motion-planner \
    ros-humble-control-toolbox \
    ros-humble-controller-interface \
    ros-humble-kinematics-interface \
    ros-dev-tools

# Set up user
ARG UID GID USERNAME GROUPNAME PASSWORD
RUN groupadd -g $GID $GROUPNAME && \
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USERNAME && \
    echo $USERNAME:$PASSWORD | chpasswd && \
    echo "$USERNAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME/workspace

COPY requirements.txt /home/$USERNAME/
RUN pip install --upgrade --no-cache-dir -r /home/$USERNAME/requirements.txt

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source ~/workspace/install/setup.bash" >> /home/$USERNAME/.bashrc

# エントリーポイントを設定
CMD ["bash"]
