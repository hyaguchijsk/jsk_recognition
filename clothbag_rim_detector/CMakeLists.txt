cmake_minimum_required(VERSION 2.8.3)
project(clothbag_rim_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS cv_bridge image_geometry pcl pcl_ros roscpp rospy std_msgs)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
# find_package(catkin REQUIRED COMPONENTS cv_bridge image_geometry pcl pcl_ros roscpp rospy std_msgs)
# find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_msgs cv_bridge image_geometry roscpp rospy std_msgs)
find_package(catkin REQUIRED COMPONENTS tf cv_bridge image_geometry roscpp rospy std_msgs)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES detect_rim
#  CATKIN_DEPENDS OpenCV tf cv_bridge image_geometry pcl pcl_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# PCL

message("*********")
message(${catkin_INCLUDE_DIRS})
include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp library
# add_library(detect_rim
#   src/${PROJECT_NAME}/detect_rim.cpp
# )

## Declare a cpp executable
# add_executable(detect_rim_node src/detect_rim_node.cpp)
add_executable(coordinate_transform src/coordinate_transform.cpp)
#add_executable(sample_point_out src/sample_point_out.cpp)
#add_executable(create_pc src/create_pc.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(detect_rim_node detect_rim_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(detect_rim_node
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(coordinate_transform ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(coordinate_transform ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
#target_link_libraries(sample_point_out ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(create_pc ${catkin_LIBRARIES} ${OpenCV_LIBS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS coordinate_transform
   ARCHIVE DESTINATION share/${PROJECT_NAME}/
   LIBRARY DESTINATION share/${PROJECT_NAME}/
   RUNTIME DESTINATION share/${PROJECT_NAME}/
 )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_detect_rim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
