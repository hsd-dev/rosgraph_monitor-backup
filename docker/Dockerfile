FROM osrf/ros:melodic-desktop-bionic

RUN apt-get update \
    && apt-get install -y git \
                          python-catkin-tools                           

RUN mkdir -p /graph_ws/src 
WORKDIR /graph_ws/src
RUN git clone --depth=1 -b nav_observer https://github.com/ipa-hsd/rosgraph_monitor/
RUN git clone --depth=1 -b SoSymPaper https://github.com/ipa-nhg/ros_graph_parser

WORKDIR /graph_ws/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/melodic/local_setup.sh \
    && catkin build \
    && . devel/local_setup.sh
