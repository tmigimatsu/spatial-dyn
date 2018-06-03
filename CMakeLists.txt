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

#########
# Eigen #
#########

set(EIGEN3_INCLUDE_DIR ${SPATIAL_DYN_LIB_DIR}/eigen.git)
set(SPATIAL_DYN_INCLUDE_DIRS ${SPATIAL_DYN_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} src)
add_subdirectory(${EIGEN3_INCLUDE_DIR})
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
set(SPATIAL_DYN_EXTERNAL_LIBS ${SPATIAL_DYN_EXTERNAL_LIBS} rbdl rbdl_urdfreader)
set(SPATIAL_DYN_INCLUDE_DIRS ${SPATIAL_DYN_INCLUDE_DIRS} ${RBDL_INCLUDE_DIRS})

# #############
# # Cpp_redis #
# #############

# set(CPP_REDIS_LIB_DIR ${SPATIAL_DYN_LIB_DIR}/cpp_redis.git)
# add_subdirectory(${CPP_REDIS_LIB_DIR})
# set(CPP_REDIS_INCLUDE_DIRS
#     ${CPP_REDIS_LIB_DIR}/includes
#     ${CPP_REDIS_LIB_DIR}/deps/include
#     ${CPP_REDIS_LIB_DIR}/tacopie/includes)
# set(SPATIAL_DYN_PUBLIC_LIBS ${SPATIAL_DYN_PUBLIC_LIBS} cpp_redis)
# set(SPATIAL_DYN_INCLUDE_DIRS ${SPATIAL_DYN_INCLUDE_DIRS} ${CPP_REDIS_INCLUDE_DIRS})

######################
# SpatialDyn library #
######################

# Include directories
include_directories(${SPATIAL_DYN_INCLUDE_DIRS})

# Create library
# set(SPATIAL_DYN_LIB spatial-dyn)
# set(SPATIAL_DYN_SRC
#     ${SPATIAL_DYN_SRC_DIR}/src/main.cc)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(${CMAKE_PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc)
add_executable(${CMAKE_PROJECT_NAME}_tests ${CMAKE_CURRENT_SOURCE_DIR}/src/tests.cc)

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