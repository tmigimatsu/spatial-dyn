############################################################
# CMakeLists for the spatial_dyn library
#
# Copyright 2018. All Rights Reserved.
#
# Created: June 7, 2018
# Authors: Toki Migimatsu
############################################################

# Require 3.6 to support pkg_check_modules IMPORTED_TARGET
cmake_minimum_required(VERSION 3.6)

# Define project
project(spatial_dyn VERSION 1.2.0 LANGUAGES CXX)

# Define options
option(BUILD_CPP "Build C++ library" ON)
option(BUILD_PYTHON "Build Python library" ON)
option(BUILD_MATLAB "Build Matlab library" OFF)

# Set CMake flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON) # TODO: Remove
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# Define directories
set(SPATIAL_DYN_LIB spatial_dyn)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(LIB_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(LIB_EXTERNAL_DIR ${PROJECT_SOURCE_DIR}/external)
set(LIB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(LIB_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

# Create spatial_dyn library
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
    set(EXT_PACKAGE_VERSION ${ARGV1})
    if(EXISTS ${LIB_CMAKE_DIR}/Find${PACKAGE_NAME}.cmake.in)
        configure_file(${LIB_CMAKE_DIR}/Find${PACKAGE_NAME}.cmake.in
            ${CMAKE_BINARY_DIR}/Find${PACKAGE_NAME}.cmake @ONLY)
    endif()
    configure_file(${LIB_CMAKE_DIR}/${PACKAGE_NAME}-CMakeLists.txt.in
        ${CMAKE_BINARY_DIR}/${PACKAGE_NAME}/CMakeLists.txt @ONLY)
    execute_process(COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
endfunction()

find_package(tinyxml2 6.2.0 QUIET)
if(NOT tinyxml2_FOUND)
    download_package(tinyxml2 6.2.0)
    set(tinyxml2_DIR "${CMAKE_BINARY_DIR}/tinyxml2/build")
    find_package(tinyxml2 QUIET NO_DEFAULT_PATH)
endif()

find_package(ctrl_utils 1.2.0 QUIET)
if(NOT ctrl_utils_FOUND)
    download_package(ctrl_utils 1.2.0)
    find_package(ctrl_utils 1.2.0 REQUIRED)
endif()

target_link_libraries(${SPATIAL_DYN_LIB}
    PUBLIC
        ctrl_utils::ctrl_utils
    PRIVATE
        tinyxml2_static
)

# Set include directories
target_include_directories(${SPATIAL_DYN_LIB}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>
    PRIVATE
        ${LIB_INCLUDE_DIR}/spatial_dyn
        ${LIB_SOURCE_DIR})

# Attach sources
target_sources(${SPATIAL_DYN_LIB} PRIVATE
    ${LIB_SOURCE_DIR}/algorithms/discrete_dynamics.cc
    ${LIB_SOURCE_DIR}/algorithms/dynamics_derivatives.cc
    ${LIB_SOURCE_DIR}/algorithms/forward_dynamics.cc
    ${LIB_SOURCE_DIR}/algorithms/forward_kinematics.cc
    ${LIB_SOURCE_DIR}/algorithms/inverse_dynamics.cc
    ${LIB_SOURCE_DIR}/algorithms/inverse_kinematics.cc
    ${LIB_SOURCE_DIR}/algorithms/opspace_dynamics.cc
    ${LIB_SOURCE_DIR}/algorithms/opspace_dynamics_aba.cc
    ${LIB_SOURCE_DIR}/algorithms/simulation.cc
    ${LIB_SOURCE_DIR}/structs/articulated_body.cc
    ${LIB_SOURCE_DIR}/structs/graphics.cc
    ${LIB_SOURCE_DIR}/structs/joint.cc
    ${LIB_SOURCE_DIR}/structs/options.cc
    ${LIB_SOURCE_DIR}/structs/rigid_body.cc
    ${LIB_SOURCE_DIR}/parsers/json.cc
    ${LIB_SOURCE_DIR}/parsers/urdf.cc
    ${LIB_SOURCE_DIR}/parsers/yaml.cc
)

# Use GNUInstallDirs to install libraries into correct locations on all platforms
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
install(DIRECTORY ${LIB_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

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
configure_package_config_file(${LIB_CMAKE_DIR}/${SPATIAL_DYN_CONFIG}.cmake.in
    ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}.cmake
    INSTALL_DESTINATION ${SPATIAL_DYN_CONFIG_INSTALL_DIR})

# Install config script files
install(FILES
    ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}.cmake
    ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}Version.cmake
    DESTINATION ${SPATIAL_DYN_CONFIG_INSTALL_DIR}
)

# Build tests
option(BUILD_TESTING "" OFF)
include(CTest)
if(BUILD_TESTING)
    add_subdirectory(test)
endif()

# Build Python wrapper
if(BUILD_PYTHON)
    add_subdirectory(${LIB_SOURCE_DIR}/python)
endif()

# Build Matlab wrapper
if(BUILD_MATLAB)
    add_subdirectory(${LIB_SOURCE_DIR}/matlab)
endif()

export(TARGETS ${SPATIAL_DYN_LIB}
    NAMESPACE ${SPATIAL_DYN_LIB}::
    FILE ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_TARGETS}.cmake)

# Register package in user registry
if(BUILD_CPP)
    export(PACKAGE ${SPATIAL_DYN_LIB})
endif()

