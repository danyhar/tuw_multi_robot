FROM registry.auto.tuwien.ac.at/roblab/docker/focal/galactic-tuw-bridge

ENV ROS1_DISTRO noetic
ENV ROS1_WS /root/ros1_ws

ENV ROS2_DISTRO galactic
ENV ROS2_WS /root/ros2_ws

ENV BRIDGE_WS /root/bridge_ws

# Create ROS1 workspace and clone the message repo
WORKDIR ${ROS1_WS}/src

##### Add more custom messages here (ROS 1) #####
COPY ./docker/ros1/RouteTable.msg ./tuw_msgs/tuw_multi_robot_msgs/msg
COPY ./docker/ros1/RouterStatus.msg ./tuw_msgs/tuw_multi_robot_msgs/msg
COPY ./docker/ros1/CMakeLists.txt ./tuw_msgs/tuw_multi_robot_msgs/

# Build the ROS1 WS
WORKDIR ${ROS1_WS}
RUN /bin/bash -c   "unset ROS_DISTRO && \
                    source /opt/ros/${ROS1_DISTRO}/setup.bash && \
                    catkin_make"


# Create ROS2 workspace and clone the message repo
WORKDIR  ${ROS2_WS}/src

##### Add more custom messages here (ROS 2) #####
COPY ./docker/ros2/RouteTable.msg ./tuw_msgs/tuw_multi_robot_msgs/msg
COPY ./docker/ros2/RouterStatus.msg ./tuw_msgs/tuw_multi_robot_msgs/msg
COPY ./docker/ros2/CMakeLists.txt ./tuw_msgs/tuw_multi_robot_msgs/


# Build the ROS2 WS
WORKDIR ${ROS2_WS}/
RUN /bin/bash -c   "unset ROS_DISTRO && \
                    source /opt/ros/${ROS2_DISTRO}/local_setup.bash && \
                    colcon build --symlink-install"


# Create Bridge workspace and clone the message repo
WORKDIR ${BRIDGE_WS}/src

# Build the Bridge WS
WORKDIR ${BRIDGE_WS}
RUN /bin/bash -c   "unset ROS_DISTRO && \
                    source /opt/ros/${ROS1_DISTRO}/setup.bash && \
                    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
                    source ~/ros1_ws/devel/setup.bash && \
                    source ~/ros2_ws/install/setup.bash && \
                    colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"
WORKDIR /

# Setup entrypoint
#COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]