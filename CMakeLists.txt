############################################################
# CMakeLists for the SpatialDyn library
#
# Copyright 2018. All Rights Reserved.
#
# Created: May 7, 2018
# Authors: Toki Migimatsu
############################################################

# Require 3.6 to support pkg_check_modules IMPORTED_TARGET
cmake_minimum_required(VERSION 3.6)

# Define project
project(spatial_dyn VERSION 1.0.0 LANGUAGES CXX)

# Set CMake flags
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON) # TODO: Remove
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# Define directories
set(SPATIAL_DYN_LIB SpatialDyn)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(SPATIAL_DYN_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(SPATIAL_DYN_EXT_DIR ${PROJECT_SOURCE_DIR}/external)
set(SPATIAL_DYN_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(SPATIAL_DYN_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# Create SpatialDyn library
add_library(${SPATIAL_DYN_LIB} SHARED)

# Create namespaced alias for use inside the build tree
add_library(${SPATIAL_DYN_LIB}::${SPATIAL_DYN_LIB} ALIAS ${SPATIAL_DYN_LIB})

# Set compiler options
target_compile_options(${SPATIAL_DYN_LIB} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    $<$<CONFIG:RELEASE>:-O3>
)

# Find dependencies
function(download_package PACKAGE_NAME)
    configure_file(${SPATIAL_DYN_CMAKE_DIR}/${PACKAGE_NAME}-CMakeLists.txt.in
        ${CMAKE_BINARY_DIR}/${PACKAGE_NAME}/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
endfunction()

# Export for build tree
configure_file(${SPATIAL_DYN_CMAKE_DIR}/Findcpp_redis.cmake.in
    ${CMAKE_BINARY_DIR}/Findcpp_redis.cmake @ONLY)
configure_file(${SPATIAL_DYN_CMAKE_DIR}/FindEigen3.cmake.in
    ${CMAKE_BINARY_DIR}/FindEigen3.cmake)
configure_file(${SPATIAL_DYN_CMAKE_DIR}/Findnlohmann_json.cmake.in
    ${CMAKE_BINARY_DIR}/Findnlohmann_json.cmake)


find_package(Eigen3 3.3.5 QUIET)
if(NOT Eigen3_FOUND)
    download_package(Eigen3)
    find_package(Eigen3 3.3.5 REQUIRED PATHS ${CMAKE_BINARY_DIR}/Eigen3/build)
endif()

find_package(cpp_redis 4.3.1 QUIET)
if(NOT cpp_redis_FOUND)
    download_package(cpp_redis)
    list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_BINARY_DIR})
    find_package(cpp_redis 4.3.1 MODULE REQUIRED)
    list(REMOVE_AT CMAKE_MODULE_PATH 0)
endif()

find_package(tinyxml2 6.2 QUIET)
if(NOT tinyxml2_FOUND)
    download_package(tinyxml2)
    set(tinyxml2_DIR "${CMAKE_BINARY_DIR}/tinyxml2/build")
    find_package(tinyxml2 QUIET NO_DEFAULT_PATH)
endif()

find_package(nlohmann_json 3.2 QUIET)
if(NOT nlohmann_json_FOUND)
    download_package(nlohmann_json)
    find_package(nlohmann_json 3.2 REQUIRED PATHS ${CMAKE_BINARY_DIR}/nlohmann_json/build)
endif()

target_link_libraries(${SPATIAL_DYN_LIB}
    PUBLIC
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
    INTERFACE
        cpp_redis::cpp_redis
    PRIVATE
        tinyxml2_static
)

# Set include directories
target_include_directories(${SPATIAL_DYN_LIB}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${SPATIAL_DYN_INCLUDE_DIR}>
    PRIVATE
        ${SPATIAL_DYN_INCLUDE_DIR}/SpatialDyn)

# Attach sources
target_sources(${SPATIAL_DYN_LIB} PRIVATE
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

# Use GNUInstalDirs to install ibraries into correct locations on all platforms
include(GNUInstallDirs)

set(SPATIAL_DYN_TARGETS ${SPATIAL_DYN_LIB}Targets)
set(SPATIAL_DYN_CONFIG ${SPATIAL_DYN_LIB}Config)
set(SPATIAL_DYN_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${SPATIAL_DYN_LIB})

# Install the library
install(TARGETS ${SPATIAL_DYN_LIB}
    EXPORT ${SPATIAL_DYN_TARGETS}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the headers
install(DIRECTORY ${SPATIAL_DYN_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export targets to Targets.cmake
install(EXPORT ${SPATIAL_DYN_TARGETS}
    DESTINATION ${SPATIAL_DYN_CONFIG_INSTALL_DIR}
    NAMESPACE ${SPATIAL_DYN_LIB}::
    FILE ${SPATIAL_DYN_TARGETS}.cmake
)

# Create ConfigVersion.cmake
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}Version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Create Config.cmake
configure_package_config_file(${SPATIAL_DYN_CMAKE_DIR}/${SPATIAL_DYN_CONFIG}.cmake.in
    ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}.cmake
    INSTALL_DESTINATION ${SPATIAL_DYN_CONFIG_INSTALL_DIR})

# Install config script files
install(FILES
    ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}.cmake
    ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}Version.cmake
    DESTINATION ${SPATIAL_DYN_CONFIG_INSTALL_DIR}
)

# Build tests
include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# Build python wrapper
add_subdirectory(${SPATIAL_DYN_SRC_DIR}/python)

export(TARGETS ${SPATIAL_DYN_LIB}
    NAMESPACE ${SPATIAL_DYN_LIB}::
    FILE ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_TARGETS}.cmake)

# Register package in user registry
export(PACKAGE ${SPATIAL_DYN_LIB})

