FROM ros:noetic-ros-core
ENV PYTHONUNBUFFERED=1

WORKDIR /home/
RUN \
  apt-get update && \
  apt-get install -y \
  build-essential \
  python3-osrf-pycommon \
  python3-catkin-tools \
  python3-rosdep \
  ros-noetic-gazebo-ros-pkgs

RUN mkdir -p catkin_ws/src
COPY ./ catkin_ws/src/mission_planner

WORKDIR /home/catkin_ws/
RUN \
  apt-get update && \
  apt-get install -y \
  git \
  libeigen3-dev \
  libgmp-dev \
  libgmpxx4ldbl \
  libmpfr-dev \
  libboost-dev \
  libboost-thread-dev \
  libtbb-dev \
  python3-dev \
  python3-pip
RUN git clone https://github.com/PyMesh/PyMesh.git
WORKDIR /home/catkin_ws/PyMesh
RUN git submodule update --init
RUN pip install -r /home/catkin_ws/PyMesh/python/requirements.txt
RUN ln -s /usr/bin/python3 /usr/bin/python
RUN python3 setup.py build
RUN python3 setup.py install

WORKDIR /home/catkin_ws/
RUN \
  rosdep init && \
  rosdep update && \
  rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

CMD /home/catkin_ws/src/mission_planner/entrypoint.sh