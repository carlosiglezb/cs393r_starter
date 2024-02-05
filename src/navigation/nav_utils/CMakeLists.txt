PROJECT(nav_utils)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

SET(HEADERS
        src/path_score_manager.h
        src/curved_path.h)

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(GTEST "gtest" REQUIRED)

INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(tests)

FILE(GLOB SOURCES src/*)
#ADD_EXECUTABLE(nav_utils
#        src/path_score_manager.cc
#        src/curved_path.cc)
#ADD_LIBRARY(nav_utils
#        src/path_score_manager.cc
#        src/curved_path.cc
#        ${HEADERS})
ADD_LIBRARY(nav_utils ${SOURCES})

ADD_EXECUTABLE(path_scores_test tests/path_score_test.cpp)
TARGET_LINK_LIBRARIES(path_scores_test ${GTEST_LIBRARIES} nav_utils)

ADD_EXECUTABLE(curved_path_test tests/curved_path_test.cpp)
TARGET_LINK_LIBRARIES(curved_path_test ${GTEST_LIBRARIES} nav_utils)

ADD_EXECUTABLE(obstacle_detection_test tests/car_obstacle_test.cpp)
TARGET_LINK_LIBRARIES(obstacle_detection_test ${GTEST_LIBRARIES} nav_utils)
