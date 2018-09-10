############################################################
# CMakeLists for the SpatialDyn library
#
# Copyright 2018. All Rights Reserved.
#
# Created: May 7, 2018
# Authors: Toki Migimatsu
############################################################

cmake_minimum_required(VERSION 3.5)

# Project definition
project(spatial_dyn)

# Compiler flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_REL "-O3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SPATIAL_DYN_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
set(SPATIAL_DYN_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SPATIAL_DYN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SPATIAL_DYN_INCLUDE_DIRS ${SPATIAL_DYN_INCLUDE_DIR}/SpatialDyn ${SPATIAL_DYN_LIB_DIR})
set(SPATIAL_DYN_PUBLIC_INCLUDE_DIRS ${SPATIAL_DYN_INCLUDE_DIR})

#########
# Eigen #
#########

set(EIGEN3_INCLUDE_DIR ${SPATIAL_DYN_LIB_DIR}/eigen.git)
set(SPATIAL_DYN_INCLUDE_DIRS ${SPATIAL_DYN_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
set(SPATIAL_DYN_PUBLIC_INCLUDE_DIRS ${SPATIAL_DYN_PUBLIC_INCLUDE_DIRS} include/eigen3)

########
# RBDL #
########

set(RBDL_LIB_DIR ${SPATIAL_DYN_LIB_DIR}/rbdl)
# set(RBDL_BUILD_ADDON_URDFREADER ON CACHE BOOL "Build the (experimental) urdf reader")
set(RBDL_USE_ROS_URDF_LIBRARY OFF CACHE BOOL "Use the URDF library provided by ROS")
add_subdirectory(${RBDL_LIB_DIR})
set(RBDL_INCLUDE_DIRS
    ${RBDL_LIB_DIR}/include
    ${RBDL_LIB_DIR}/addons
    ${CMAKE_BINARY_DIR}/lib/rbdl/include)
# set(SPATIAL_DYN_EXTERNAL_LIBS ${SPATIAL_DYN_EXTERNAL_LIBS} rbdl rbdl_urdfreader)
set(SPATIAL_DYN_EXTERNAL_LIBS ${SPATIAL_DYN_EXTERNAL_LIBS} rbdl)
set(SPATIAL_DYN_INCLUDE_DIRS ${SPATIAL_DYN_INCLUDE_DIRS} ${RBDL_INCLUDE_DIRS})

#############
# Cpp_redis #
#############

set(CPP_REDIS_LIB_DIR ${SPATIAL_DYN_LIB_DIR}/cpp_redis.git)
set(BUILD_EXAMPLES OFF)
set(BUILD_TESTS OFF)
add_subdirectory(${CPP_REDIS_LIB_DIR})
set(CPP_REDIS_INCLUDE_DIRS
    ${CPP_REDIS_LIB_DIR}/includes
    ${CPP_REDIS_LIB_DIR}/deps/include
    ${CPP_REDIS_LIB_DIR}/tacopie/includes)
set(SPATIAL_DYN_PUBLIC_LIBS ${SPATIAL_DYN_PUBLIC_LIBS} cpp_redis)
set(SPATIAL_DYN_INCLUDE_DIRS ${SPATIAL_DYN_INCLUDE_DIRS} ${CPP_REDIS_INCLUDE_DIRS})

############
# TinyXML2 #
############

set(TINYXML_LIB_DIR ${SPATIAL_DYN_LIB_DIR}/tinyxml2.git)
add_subdirectory(${TINYXML_LIB_DIR})
set(TINYXML_INCLUDE_DIRS ${TINYXML_LIB_DIR})
set(SPATIAL_DYN_INCLUDE_DIRS ${SPATIAL_DYN_INCLUDE_DIRS} ${TINYXML_INCLUDE_DIRS})
set(SPATIAL_DYN_EXTERNAL_LIBS ${SPATIAL_DYN_EXTERNAL_LIBS} tinyxml2)

######################
# SpatialDyn library #
######################

# Include directories
include_directories(${SPATIAL_DYN_INCLUDE_DIRS})

# Create library
# set(SPATIAL_DYN_LIB spatial-dyn)
set(SPATIAL_DYN_SRC
    ${SPATIAL_DYN_SRC_DIR}/algorithms/forward_dynamics.cc
    ${SPATIAL_DYN_SRC_DIR}/algorithms/forward_kinematics.cc
    ${SPATIAL_DYN_SRC_DIR}/algorithms/inverse_dynamics.cc
    ${SPATIAL_DYN_SRC_DIR}/algorithms/opspace_dynamics.cc
    ${SPATIAL_DYN_SRC_DIR}/algorithms/opspace_dynamics_aba.cc
    ${SPATIAL_DYN_SRC_DIR}/algorithms/simulation.cc
    ${SPATIAL_DYN_SRC_DIR}/structs/articulated_body.cc
    ${SPATIAL_DYN_SRC_DIR}/structs/joint.cc
    ${SPATIAL_DYN_SRC_DIR}/structs/rigid_body.cc
    ${SPATIAL_DYN_SRC_DIR}/parsers/json.cc
    ${SPATIAL_DYN_SRC_DIR}/parsers/urdf.cc
    ${SPATIAL_DYN_SRC_DIR}/utils/timer.cc
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(${CMAKE_PROJECT_NAME} ${SPATIAL_DYN_SRC_DIR}/main.cc ${SPATIAL_DYN_SRC})
add_executable(${CMAKE_PROJECT_NAME}_tests ${CMAKE_CURRENT_SOURCE_DIR}/test/tests.cc ${SPATIAL_DYN_SRC})

# add_library(${SPATIAL_DYN_LIB} SHARED ${SPATIAL_DYN_SRC})
# target_include_directories(${SPATIAL_DYN_LIB} INTERFACE
#     $<BUILD_INTERFACE:${SPATIAL_DYN_INCLUDE_DIRS}>
#     $<INSTALL_INTERFACE:${SPATIAL_DYN_PUBLIC_INCLUDE_DIRS}>)
target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE ${SPATIAL_DYN_EXTERNAL_LIBS}
    PUBLIC ${SPATIAL_DYN_PUBLIC_LIBS})
target_link_libraries(${CMAKE_PROJECT_NAME}_tests
    PRIVATE ${SPATIAL_DYN_EXTERNAL_LIBS}
    PUBLIC ${SPATIAL_DYN_PUBLIC_LIBS})
# set_target_properties(${SPATIAL_DYN_LIB} PROPERTIES
#     LIBRARY_OUTPUT_DIRECTORY ${SPATIAL_DYN_LIB_DIR}
#     SUFFIX ".so")

# # Install library
# set(SPATIAL_DYN_CONFIG spatial-dyn-config)
# install(TARGETS ${SPATIAL_DYN_LIB}
#         EXPORT ${SPATIAL_DYN_CONFIG}
#         LIBRARY DESTINATION lib)
# install(DIRECTORY ${SPATIAL_DYN_SRC_DIR}/
#         DESTINATION include
#         FILES_MATCHING PATTERN "*.h*")

# # Make library available via find_package
# set(SPATIAL_DYN_LIBRARIES ${SPATIAL_DYN_LIB} ${SPATIAL_DYN_PUBLIC_LIBS})
# string(REGEX REPLACE "([^;]+)" "${CMAKE_INSTALL_PREFIX}/\\1"
#     SPATIAL_DYN_CONF_INCLUDE_DIRS "${SPATIAL_DYN_PUBLIC_INCLUDE_DIRS}")
# configure_package_config_file(${SPATIAL_DYN_CONFIG}.cmake.in
#     ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}.cmake
#     INSTALL_DESTINATION lib/cmake)
# write_basic_package_version_file(
#     ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}-version.cmake
#     VERSION 0.1.0
#     COMPATIBILITY SameMajorVersion)
# install(FILES ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}.cmake
#               ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}-version.cmake
#         DESTINATION lib/cmake)
# export(PACKAGE ${SPATIAL_DYN_LIB})

##################
# Subdirectories #
##################

# add_subdirectory(tests)
add_subdirectory(${SPATIAL_DYN_SRC_DIR}/python)
