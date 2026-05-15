FROM osrf/ros:lyrical-desktop

# install gazebo jetty and stuff
RUN apt-get update && apt-get install -y \
    gz-sim10-server \
    ros-lyrical-ros-gz \
    ros-lyrical-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*