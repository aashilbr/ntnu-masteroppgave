FROM ubuntu:20.04
WORKDIR /home/

#Do this as root, before changing user
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
    python3 \
    python3-pip \
    python3-rospy \
    git \
    ros-desktop-full-python-dev

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

WORKDIR /home/
RUN git clone https://github.com/PyMesh/PyMesh.git
WORKDIR /home/PyMesh/
RUN git submodule update --init
RUN pip install -r /home/PyMesh/python/requirements.txt
RUN ln -s /usr/bin/python3 /usr/bin/python
RUN python3 setup.py build
RUN python3 setup.py install

WORKDIR /home/
COPY ./requirements.txt ./requirements.txt
RUN pip install -r requirements.txt

# Change user
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

WORKDIR /home/workspace
COPY ./ ./

CMD /home/workspace/entrypoint.sh