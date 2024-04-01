PROJECT(global_planner)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

FIND_PACKAGE(PkgConfig)

FILE(GLOB SOURCES rrt.cc rrt.h)

ADD_LIBRARY(global_planner ${SOURCES})

ADD_EXECUTABLE(rrt_test main_rrt.cpp)
TARGET_LINK_LIBRARIES(rrt_test global_planner)
