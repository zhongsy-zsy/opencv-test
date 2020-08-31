cmake_minimum_required(VERSION 3.1.0)

project(realsense)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)

add_library(${PROJECT_NAME}
 d435_driver.cc
)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${realsense2_INCLUDE_DIRS})

add_executable(main class_main.cc )

target_link_libraries( main ${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY}  -lpthread -lm )


