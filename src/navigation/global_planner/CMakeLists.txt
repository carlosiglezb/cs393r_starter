PROJECT(global_planner)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

FIND_PACKAGE(PkgConfig)

FILE(GLOB SOURCES rrt.cc rrt.h rrt_star.cc rrt_star.h geometry_tools.hpp)

FIND_LIBRARY(amrl_shared_lib amrl_shared_lib HINTS ${CMAKE_SOURCE_DIR}/../ut_automata/lib)
ADD_LIBRARY(global_planner ${SOURCES})
TARGET_LINK_LIBRARIES(global_planner ${amrl_shared_lib})

ADD_EXECUTABLE(rrt_test main_rrt.cpp)
TARGET_LINK_LIBRARIES(rrt_test global_planner)

ADD_EXECUTABLE(rrt_star_test main_rrt_star.cpp)
TARGET_LINK_LIBRARIES(rrt_star_test global_planner ${amrl_shared_lib})
