PROJECT(particle_filter_utils)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#SET(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror")
SET(CMAKE_CXX_STANDARD 17)

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(GTEST "gtest" REQUIRED)

INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(tests)

FILE(GLOB SOURCES src/*)

ADD_LIBRARY(particle_filter_utils ${SOURCES})

ADD_EXECUTABLE(motion_model_test tests/motion_model_test.cpp)
TARGET_LINK_LIBRARIES(motion_model_test ${GTEST_LIBRARIES} particle_filter_utils)

ADD_EXECUTABLE(resample_main_test tests/resample_main.cpp)
TARGET_LINK_LIBRARIES(resample_main_test particle_filter_utils)
