cmake_minimum_required(VERSION 3.1.0)

project(realsense)

set( CMAKE_CXX_COMPILER "g++" )
# set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set(CMAKE_BUILD_TYPE Debug CACHE STRING "set build type to debug")

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Found OpenMP")
ENDIF()

add_library(${PROJECT_NAME}
 d435_driver.cc
 common_op.cc
)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${realsense2_INCLUDE_DIRS}
)

add_executable(main class_main.cc )
add_executable(cali calibration.cc )
add_executable(test_visit test_visit.cc )
add_executable(get_d435_info get_d435_info.cc )
add_executable(moving moving_callback.cc )
add_executable(back_project backproject.cc )



# add_executable(yuzhi common_op.cc )


target_link_libraries( main ${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY}  -lpthread -lm )
target_link_libraries( cali ${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY}  -lpthread -lm )
target_link_libraries( test_visit ${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY}  -lpthread -lm )
target_link_libraries( get_d435_info ${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY}  -lpthread -lm )
target_link_libraries( moving ${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY}  -lpthread -lm )
target_link_libraries( back_project ${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY}  -lpthread -lm )



# target_link_libraries( yuzhi ${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY}  -lpthread -lm )

