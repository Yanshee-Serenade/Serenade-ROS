FROM lucasmogsan/orbslam3_ros:latest
LABEL maintainer="subcat2077@gmail.com"

USER root

RUN catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1

COPY ubt_msgs ./src/ubt_msgs
RUN catkin build ubt_msgs

RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc
COPY ros_entrypoint.sh /ros_entrypoint.sh
