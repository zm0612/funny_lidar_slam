FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

ENV CATKIN_WS=/root/funny_lidar_slam_ws

RUN apt-get update -y

RUN apt-get install -y libgoogle-glog-dev \
    libgflags-dev \
    libgtest-dev \
    git

# install g2o dependences
RUN apt install -y libeigen3-dev \
        libspdlog-dev \
        libsuitesparse-dev \
        qtdeclarative5-dev \
        qt5-qmake \
        libqglviewer-dev-qt5

# install g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && mkdir build && cd build && \
    cmake .. && make -j && make install &&\
    rm -rf ../../g2o

RUN mkdir -p ${CATKIN_WS}/src/funny_lidar_slam
COPY . ${CATKIN_WS}/src/funny_lidar_slam

WORKDIR $CATKIN_WS

RUN rm -rf /var/lib/apt/lists/*
