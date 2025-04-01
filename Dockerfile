

FROM ubuntu:20.04


LABEL maintainer="f20171569@hyderabad.bits-pilani.ac.in"


# ROS2 Installation Starts
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
   ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
   apt-get update && \
   apt-get install -q -y --no-install-recommends tzdata && \
   rm -rf /var/lib/apt/lists/*


# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
   bash-completion \
   cmake \
   dirmngr \
   git \
   gnupg2 \
   libssl-dev \
   lsb-release \
   python3-pip \
   wget \
   curl \
   && rm -rf /var/lib/apt/lists/*


# locale
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8


# Setup Sources
RUN apt update && apt install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg


RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# install tools
RUN apt-get update && apt-get install --no-install-recommends -y \
   build-essential \
   git \
   python3-colcon-common-extensions \
   python3-colcon-mixin \
   python3-rosdep \
   python3-vcstool \
   python3.8-venv \
   xboxdrv \
   && rm -rf /var/lib/apt/lists/*


# install python packages
RUN pip3 install -U \
   argcomplete


# Install pytest and its plugins separately to avoid conflicts
RUN pip3 install pytest==6.2.4 pytest-repeat==0.9.1 pytest-rerunfailures==9.1.1


# Verify pytest installation
RUN pytest --version
# Install ROS2 Foxy
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-foxy-desktop


# Install missing dependencies for turtlesim
RUN apt-get update && apt-get install -y \
   ros-foxy-std-srvs


# Make ROS2 Workspace and build tutorials
# Development Dependency Workspace for tutorials




# # not wokring currently but also not be needed
# RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel


# Clone ROS tutorials (currently not wokring)
RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel /root/ros_tutorials


# Setup rosdep using /root directory
RUN rosdep init && rosdep update --rosdistro=foxy && \
   rosdep install -i --from-path /root/ros_tutorials --rosdistro foxy -y




# Create and activate a Python virtual environment
RUN python3 -m venv /opt/ros2_venv
RUN /opt/ros2_venv/bin/pip install --upgrade pip


# Activate the virtual environment and install Python packages
RUN /opt/ros2_venv/bin/pip install -U argcomplete pytest==6.2.4 pytest-repeat==0.9.1 pytest-rerunfailures==9.1.1 pygame
#install pygame


# Use the virtual environment for pytest
RUN /opt/ros2_venv/bin/pytest --version

