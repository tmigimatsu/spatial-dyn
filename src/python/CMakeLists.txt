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

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/spatialdyn)

find_package(pybind11 REQUIRED)
add_subdirectory(${PYBIND11_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/pybind11)

pybind11_add_module(spatialdyn spatial_dyn.cc eigen.cc ${SPATIAL_DYN_SRC})
target_link_libraries(spatialdyn PUBLIC pybind11::pybind11 spatial_dyn::spatial_dyn)

pybind11_add_module(eigen eigen.cc ${SPATIAL_DYN_SRC})
target_link_libraries(eigen PUBLIC pybind11::pybind11 spatial_dyn::spatial_dyn)
