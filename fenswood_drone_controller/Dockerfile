FROM uobflightlabstarling/starling-controller-base:latest

RUN apt update
RUN apt-get install -y ros-foxy-vision-opencv python3-pip
RUN pip3 install opencv-python

COPY . /ros_ws/src/fenswood_drone_controller

RUN . /ros_ws/install/setup.sh \
    && colcon build

CMD [ "ros2", "launch", "fenswood_drone_controller", "controller.launch.xml" ]
