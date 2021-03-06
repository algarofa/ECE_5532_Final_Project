cmake_minimum_required(VERSION 2.8.3)
project(ece_5532_final)

find_package(catkin REQUIRED COMPONENTS
roscpp
tf
dynamic_reconfigure
ugv_course_libs
sensor_msgs
std_msgs
geometry_msgs
image_transport
cv_bridge
)
find_package(OpenCV REQUIRED)


catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ./PID/cpp/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(PID PID/cpp/PID.cpp)
target_link_libraries(PID ${catkin_LIBRARIES})

add_executable(follow_twist_override src/follow_twist_override.cpp)
target_link_libraries(follow_twist_override ${catkin_LIBRARIES} PID)

add_executable(follow_camera src/follow_camera.cpp)
target_link_libraries(follow_camera ${catkin_LIBRARIES} PID ${OpenCV_LIBS})

add_executable(lidar_nav src/lidar_nav.cpp)
target_link_libraries(lidar_nav ${catkin_LIBRARIES} PID)