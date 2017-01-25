# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)

# set the project name
project(TestAssignmentMakeItRoll)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# add the required cmake packages
find_package(RTF)
find_package(RTF COMPONENTS DLL)
find_package(YARP REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

# add include directories
include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})

# add required libraries
link_libraries(${RTF_LIBRARIES} ${YARP_LIBRARIES})

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

# add the source codes to build the plugin library
add_library(${PROJECT_NAME} MODULE test.cpp)
