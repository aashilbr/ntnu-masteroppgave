FROM ros:noetic-ros-core
WORKDIR /home/

RUN \
  apt-get update && \
  apt-get install -y \
  build-essential \
  python3-osrf-pycommon \
  python3-catkin-tools \
  python3-rosdep


WORKDIR /home/
RUN mkdir -p catkin_ws/src
COPY ./ catkin_ws/src/ros_noetic

WORKDIR /home/catkin_ws/
RUN \
  rosdep init && \
  rosdep update && \
  rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

#COPY ./keyboard /etc/default/keyboard
#RUN \
#  apt-get update && \
#  DEBIAN_FRONTEND=noninteractive \
#  apt-get install -y \
#  ros-noetic-rqt \
#  ros-noetic-rqt-common-plugins

CMD /home/catkin_ws/src/ros_noetic/entrypoint.sh