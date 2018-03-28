cmake_minimum_required(VERSION 2.8.3)
project(leg_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS}/bfl)
message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  visualization_msgs
  image_geometry
  image_transport
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs 
  nav_msgs
  tf  
)

generate_dynamic_reconfigure_options(cfg/LegDetector.cfg) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES leg_detector
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sensor_msgs  nav_msgs leg_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BFL_INCLUDE_DIRS}
)

add_executable(leg_detector src/leg_detector.cpp
			    src/extraction_processor.cpp
			    src/HOG_processor.cpp
			    include/tracker/tracker_kalman.cpp
			    include/detector/laser_processor.cpp
			    include/detector/calc_leg_features.cpp
			    include/svmlight/SvmLightLib.cpp
			    include/svmlight/svm_common.c
			    include/svmlight/svm_learn.c
			    include/svmlight/svm_hideo.c) 

add_dependencies(leg_detector ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_genpy ${${PROJECT_NAME}_EXPORTED_TARGETS} )						  

target_link_libraries(leg_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})
