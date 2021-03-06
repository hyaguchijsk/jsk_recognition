# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(checkerboard_detector)

find_package(catkin REQUIRED COMPONENTS roscpp rosconsole cv_bridge sensor_msgs posedetection_msgs)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(posedetection_msgs)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(checkerboard_detector src/checkerboard_detector.cpp)
target_link_libraries(checkerboard_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(checkerboard_calibration src/checkerboard_calibration.cpp)
target_link_libraries(checkerboard_calibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(checkerboard_detector    posedetection_msgs_gencpp)
add_dependencies(checkerboard_calibration posedetection_msgs_gencpp)

catkin_package(
    CATKIN_DEPENDS roscpp rosconsole cv_bridge sensor_msgs posedetection_msgs
    DEPENDS OpenCV2
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(TARGETS checkerboard_detector checkerboard_calibration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB _launch *.launch)
file(GLOB _pdf *.pdf)
install(FILES ${_launch} ${_pdf}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
