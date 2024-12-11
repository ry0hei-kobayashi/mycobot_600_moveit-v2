FROM osrf/ros:humble-desktop-full
#FROM ubuntu:22.04
#FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

#-----------------------------
# Environment Variables
#-----------------------------
ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8
# no need input key
ENV DEBIAN_FRONTEND noninteractive

SHELL ["/bin/bash", "-c"]

RUN sed -i 's@archive.ubuntu.com@ftp.jaist.ac.jp/pub/Linux@g' /etc/apt/sources.list

#-----------------------------
# Install common pkg
#-----------------------------
RUN apt-get -y update
RUN apt-get -y install git ssh python3-pip wget net-tools vim tmux xsel xclip curl make build-essential lsb-release libgl1-mesa-dev python3-wheel python3-pip python-is-python3

#-----------------------------
# Install humble pkg
#-----------------------------
ENV UBUNTU_CODENAME=jammy
#RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
#    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list
#RUN apt-get update
#RUN apt-get install -y ros-humble-desktop-full \
RUN apt-get update
RUN apt-get install -y ros-humble-moveit \
    ros-humble-controller-manager \
    ros-humble-gazebo-ros \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-setup-assistant \
    ros-humble-gazebo-ros2-control \
    ros-humble-gripper-controllers \
    ros-humble-ros-gz \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    python3-rosdep \
    python3-colcon-common-extensions \ 
    ros-humble-gazebo-ros-pkgs 

#-----------------------------
# Build Workspace
#-----------------------------
RUN mkdir -p /colcon_ws/src
WORKDIR /colcon_ws/src

COPY ./config /colcon_ws/src/mycobot_600_moveit/config
COPY ./launch /colcon_ws/src/mycobot_600_moveit/launch
COPY ./urdf /colcon_ws/src/mycobot_600_moveit/urdf
COPY ./scripts /colcon_ws/src/mycobot_600_moveit/scripts
COPY ./CMakeLists.txt /colcon_ws/src/mycobot_600_moveit/CMakeLists.txt
COPY ./package.xml /colcon_ws/src/mycobot_600_moveit/package.xml
COPY ./setup.py /colcon_ws/src/mycobot_600_moveit/setup.py
COPY ./.setup_assistant /colcon_ws/src/mycobot_600_moveit/.setup_assistant

RUN cd .. && . /opt/ros/humble/setup.bash && colcon build && . install/setup.bash
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && rosdep update

# add my tmux config to this container
RUN cd / && git clone https://github.com/ry0hei-kobayashi/tmux-conf.git && mv tmux-conf/.tmux.conf ~/

COPY ./docker/ros_entrypoint.sh /
RUN chmod 775 /ros_entrypoint.sh

CMD ["/bin/bash"]
ENTRYPOINT ["/ros_entrypoint.sh"]
