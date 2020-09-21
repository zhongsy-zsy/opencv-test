cmake_minimum_required(VERSION 3.1.0)
project("mygtest")


FIND_PACKAGE(OpenCV REQUIRED)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-std=c++11")

add_library(my_lib
  ../realsense/common_op.cc
)

include_directories(${TARGET_NAME} usr/include/gtest)
include_directories(${OpenCV_DIRS})

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME}  /usr/local/lib/libgtest_main.a /usr/local/lib/libgtest.a -lpthread -lm ${OpenCV_LIBS} my_lib)
