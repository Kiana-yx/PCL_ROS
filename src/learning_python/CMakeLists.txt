cmake_minimum_required(VERSION 3.0.2)
project(learning_python) 

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy 
std_msgs 
message_generation 
) 

generate_messages( 
DEPENDENCIES 
std_msgs 
) 

catkin_package(
# INCLUDE_DIRS include 
# LIBRARIES learning_python 
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
# DEPENDS system_lib 
) 

include_directories( 
# include 
${catkin_INCLUDE_DIRS} 
)
