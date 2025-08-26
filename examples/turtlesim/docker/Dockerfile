
# Build Arguments and Base Image
FROM osrf/ros:humble-desktop

ENV ROS_DISTRO=humble
ENV ROS_WS=/palletizer_ws
SHELL ["/bin/bash", "-c"]


# Create workspace and copy source
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Copy ROS package(s) into workspace/src
COPY perception_orientation_detectors_gl/ ${ROS_WS}/src/perception_orientation_detectors_gl
COPY hal_cameras/ ${ROS_WS}/src/hal_cameras
COPY hal_interfaces/ ${ROS_WS}/src/hal_interfaces

# Fix expired ROS GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor --batch --yes -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# Install tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    # python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN pip install src/perception_orientation_detectors_gl
RUN pip install src/hal_cameras
# no setup.py for hal_interfaces

RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Build the ROS 2 workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Auto-source environment in container shell
RUN echo "source /opt/ros/humble/setup.bash && source ${ROS_WS}/install/setup.bash" >> ~/.bashrc
