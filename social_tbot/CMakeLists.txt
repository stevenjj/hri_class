cmake_minimum_required(VERSION 2.8.3)
project(social_tbot)	

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  turtlesim
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES social_tbot
#  CATKIN_DEPENDS roscpp turtlesim
  DEPENDS
  	roscpp
  	turtlesim
  	geometry_msgs
)

add_executable(social_tbot_node src/social_tbot.cpp)
target_link_libraries(social_tbot_node ${catkin_LIBRARIES})
