from ros:melodic

ENV ROS_WORKSPACE_PATH=/root/catkin_ws
ENV ROS_PACKAGE_NAME=msp_fc_driver

# Install dependencies
RUN apt update -y && apt install -y \
    build-essential \
    python-catkin-pkg \
    python-catkin-tools \
    ros-melodic-mav-msgs \
    python-serial

# Setup catkin
RUN mkdir -p $ROS_WORKSPACE_PATH/src && \
    cd $ROS_WORKSPACE_PATH && \
    catkin init && \
    catkin config --extend /opt/ros/melodic/ && \
    echo "source $ROS_WORKSPACE_PATH/devel/setup.bash" >> ~/.bashrc && \
    /bin/bash -c "source ~/.bashrc"

WORKDIR $ROS_WORKSPACE_PATH/src

COPY ./ $ROS_WORKSPACE_PATH/src/$ROS_PACKAGE_NAME/
RUN cd $ROS_WORKSPACE_PATH && \
    catkin build

