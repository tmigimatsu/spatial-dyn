############################################################
# CMakeLists for Pybind11
#
# Copyright 2018. All Rights Reserved.
#
# Created: September 7, 2018
# Authors: Toki Migimatsu
############################################################

#############
# Pybind11 #
#############

project(spatial_dyn_python VERSION 1.2.0 LANGUAGES CXX)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/spatialdyn)

add_subdirectory(${ctrl_utils_EXTERNAL_DIR}/pybind11
    ${CMAKE_CURRENT_BINARY_DIR}/pybind11)

pybind11_add_module(spatialdyn spatial_dyn.cc eigen.cc ${SPATIAL_DYN_SRC})
target_link_libraries(spatialdyn PUBLIC
    pybind11::pybind11
    spatial_dyn::spatial_dyn)

pybind11_add_module(spatialeigen eigen.cc ${SPATIAL_DYN_SRC})
target_link_libraries(spatialeigen PUBLIC
    pybind11::pybind11
    spatial_dyn::spatial_dyn)
