# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-


# This folder contains executables that are only built if ROS is available. Executables in here are
# allowed to use core ROS function like sensor_msgs::Image or std_msgs::visualization_marker_array.
# If you want to use additional features like custom msg types or cfg parameters as well as launch
# file functionalities, you have to built these in a catkin package e.g. ros_oadrive.
#
# So the executables in here are only meant to be used for quick debugging functions where you do
# not want to install your whole ic_workspace and build a catkin_ws afterwards.
#
# If you need large adaptation functions, consider creating an oadrive_ros library.


FIND_PACKAGE(ROS COMPONENTS image_transport cv_bridge)


#########################################################

# test/test_oadrive_interface extended with basic ROS communication
ICMAKER_SET("test_oadrive_interface_ros")

ICMAKER_ADD_SOURCES(
        test_oadrive_interface_ros.cpp
)

ICMAKER_ADD_HEADERS(
)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(
        oadrive_core
        oadrive_util
        oadrive_interface
        oadrive_world
        ROS
        ROS_IMAGE_TRANSPORT
        ROS_CV_BRIDGE
)

ICMAKER_BUILD_PROGRAM()

#########################################################
