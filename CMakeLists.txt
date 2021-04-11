cmake_minimum_required(VERSION 2.8.3)
project(ECE_5532_Final_Project)

find_package(catkin REQUIRED COMPONENTS
roscpp
tf
dynamic_reconfigure
ugv_course_libs
sensor_msgs
std_msgs
geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ECE_5532_Final_Project src/follow_twist_override.cpp)
target_link_libraries(ECE_5532_Final_Project ${catkin_LIBRARIES})