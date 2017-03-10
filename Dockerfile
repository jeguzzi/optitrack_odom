FROM jeguzzi/ros:kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
   ros-kinetic-tf \
   && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/jeguzzi/optitrack_msgs.git /root/catkin_ws/src/optitrack_msgs
RUN git clone https://github.com/jeguzzi/optitrack_odom.git /root/catkin_ws/src/optitrack_odom
RUN catkin build
