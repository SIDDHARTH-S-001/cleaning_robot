FROM ros:noetic

ENV DEBIAN_FRONTEND=noninterac

RUN apt-get update && apt-get install -y \
    ros-noetic-rosserial* \
    ros-noetic-cv-bridge* \
    ros-noetic-rospy* \
    ros-noetic-gazebo*

RUN apt-get install -y libudev-dev \
    libusb*

WORKDIR /cleaning_robot_coverage /catkin_ws/src/cleaning_robot_coverage

WORKDIR /cleaning_robot_description /catkin_ws/src/cleaning_robot_description

WORKDIR /cleaning_robot_gazebo /catkin_ws/src/cleaning_robot_gazebo

WORKDIR /cleaning_robot_hardware /catkin_ws/src/cleaning_robot_hardware

WORKDIR /cleaning_robot_navigation /catkin_ws/src/cleaning_robot_navigation

WORKDIR /full_coverage_path_planner /catkin_ws/src/full_coverage_path_planner

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash \
    && catkin_make'

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

CMD bash