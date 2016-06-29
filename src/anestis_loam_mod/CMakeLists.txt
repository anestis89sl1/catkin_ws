cmake_minimum_required(VERSION 2.8.3)
project(anestis_loam_mod)
set(SURE_DIR "~/workspace/rgbdpro/sure3d/")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  rosbag
)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${SURE_DIR}include
#	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})
link_directories ( ${Boost_LIBRARY_DIRS} )
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs rosbag
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)
include(pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE()
find_package(Boost REQUIRED)

set(srcs
	${SURE_DIR}src/sure/normal_histogram.cpp
	${SURE_DIR}src/sure/octree_value.cpp
	${SURE_DIR}src/sure/configuration.cpp
	${SURE_DIR}src/sure/surflet.cpp
	${SURE_DIR}src/sure/color_surflet.cpp
	${SURE_DIR}src/sure/map2d.cpp
	${SURE_DIR}src/sure/range_image.cpp
	${SURE_DIR}src/sure/histogram_base.cpp
	${SURE_DIR}src/sure/histogram_l2.cpp
	${SURE_DIR}src/sure/histogram_emd.cpp
	${SURE_DIR}src/sure/hue_descriptor.cpp
	${SURE_DIR}src/sure/point_feature_descriptor.cpp
	${SURE_DIR}src/sure/lightness_descriptor.cpp
	${SURE_DIR}src/sure/feature.cpp
	${SURE_DIR}src/sure/sure_estimator.cpp
	)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED ${srcs})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES} -lboost_serialization)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 0.2.7) 

add_executable(An_scanRegistration src/scanRegistration.cpp)
target_link_libraries(An_scanRegistration ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} -lboost_serialization)

add_executable(An_genetic src/genetic.cpp)
target_link_libraries(An_genetic ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} -lboost_serialization)

add_executable(An_laserOdometry src/laserOdometry.cpp)
target_link_libraries(An_laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(An_laserMapping src/laserMapping.cpp)
target_link_libraries(An_laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(An_transformMaintenance src/transformMaintenance.cpp)
target_link_libraries(An_transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

## Declare a C++ library
# add_library(anestis_loam_mod
#   src/${PROJECT_NAME}/anestis_loam_mod.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(anestis_loam_mod ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(anestis_loam_mod_node src/anestis_loam_mod_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(anestis_loam_mod_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(anestis_loam_mod_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS anestis_loam_mod anestis_loam_mod_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_anestis_loam_mod.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
