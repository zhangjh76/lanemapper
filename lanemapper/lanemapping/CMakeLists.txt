cmake_minimum_required(VERSION 3.0.2)
project(lanemapper)

# cmake -DCMAKE_PYTHON_INTERPRETER=/home/seu/anaconda3/bin/python3/
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(eigen REQUIRED QUIET)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  lane_msgs
)
# find_package(Eigen REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# add_message_files(
#   DIRECTORY msg
#   FILES
#   Lane.msg
#   LaneList.msg
#   LanePoint.msg
# )

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	# ${EIGEN_INCLUDE_DIRS}
)
link_directories(
  include
)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS lane_msgs
#  INCLUDE_DIRS include
#  LIBRARIES my_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(lanemapper src/lanemapper.cpp)
add_dependencies(lanemapper ${catkin_EXPORTED_TARGETS} lane_msgs_gencpp)
target_link_libraries(lanemapper ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
