cmake_minimum_required(VERSION 2.8.3)
project(leg_follow)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES leg_follow
  CATKIN_DEPENDS geometry_msgs leg_msgs roscpp cir_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(odom_only src/odom_only.cpp
#			  src/vfh_algorithm.cpp)
#target_link_libraries(odom_only ${catkin_LIBRARIES})


add_executable(leg_follow src/leg_follow.cpp
			  src/vfh_algorithm.cpp)
target_link_libraries(leg_follow ${catkin_LIBRARIES})



#add_dependencies(leg_follow ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_genpy ${${PROJECT_NAME}_EXPORTED_TARGETS} )	
