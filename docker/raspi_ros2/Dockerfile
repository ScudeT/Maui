FROM ros:jazzy-ros-core-noble

# Setup ros user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# change given ubuntu user to ros
RUN usermod -l ${USERNAME} ubuntu \
    && usermod -d /home/${USERNAME} -m ${USERNAME} \
    && groupmod -n ${USERNAME} ubuntu

# Setup sudo for user
RUN apt-get update \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# Add user to dialout
RUN adduser ${USERNAME} dialout

# Setup XLaunch    
RUN apt-get update && \
    apt-get install -y x11-apps

# install ROS2 devealopment tools
RUN apt-get update && apt-get install -y \
    python3-flake8-blind-except \
    python3-flake8-class-newline \
    python3-flake8-deprecated \
    python3-mypy \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout \
    ros-dev-tools

RUN apt-get update && \
    apt-get install -y \
    wget \
    nano \
    tree \
    screen \
    git-all 

USER ${USERNAME}

RUN mkdir -p /home/${USERNAME}/ros2_ws/src

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/${USERNAME}/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/jazzy/" >> /home/${USERNAME}/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /home/${USERNAME}/ros2_ws/install/local_setup.bash" >> /home/${USERNAME}/.bashrc

USER root

# hardware setups ########################
RUN apt-get update && apt-get install -y \
    libgpiod-dev \
    i2c-tools \
    libi2c-dev \
    gpsd \
    gpsd-clients \
    minicom \ 
    python3-libgpiod


# allow pip to install packages without the need for a virtual environment
ARG PIP_BREAK_SYSTEM_PACKAGES=1 

RUN pip install gpsd-py3

#######################################

## ROS PACKAGES #####
RUN apt-get update && apt-get install -y ros-jazzy-gps-umd                 
RUN apt-get update && apt-get install -y ros-jazzy-rviz-satellite 
RUN apt-get update && apt-get install -y ros-jazzy-rviz-imu-plugin 
RUN apt-get update && apt-get install -y ros-jazzy-rviz-2d-overlay-plugins 
RUN apt-get update && apt-get install -y ros-jazzy-joint-state-publisher-gui
RUN apt-get update && apt-get install -y ros-jazzy-foxglove-bridge 
RUN apt-get update && apt-get install -y ros-jazzy-ros2-control 
RUN apt-get update && apt-get install -y ros-jazzy-ros2-controllers 
RUN apt-get update && apt-get install -y ros-jazzy-robot-localization
RUN apt-get update && apt-get install -y ros-jazzy-rqt ros-jazzy-rqt-common-plugins
RUN apt-get update && apt-get install -y ros-jazzy-tf2 ros-jazzy-tf2-ros

# Set up entrypoint and default command
RUN chmod 777 -R /home/${USERNAME}/ros2_ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["/bin/bash"]
