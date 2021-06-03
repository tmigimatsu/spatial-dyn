############################################################
# CMakeLists for Pybind11.
#
# Copyright 2018. All Rights Reserved.
#
# Created: September 7, 2018
# Authors: Toki Migimatsu
############################################################

project(spatialdyn)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIB_SOURCE_DIR}/spatialdyn")

ctrl_utils_add_subdirectory(pybind11)

# Create spatial-dyn wrapper.
pybind11_add_module(spatialdyn spatial_dyn.cc eigen.cc)

# Link library dependencies.
target_link_libraries(spatialdyn
  PUBLIC
    pybind11::pybind11
    spatial_dyn::spatial_dyn
)

# Create Eigen wrapper.
pybind11_add_module(spatialeigen eigen.cc)

# Link library dependencies.
target_link_libraries(spatialeigen
  PUBLIC
    pybind11::pybind11
    spatial_dyn::spatial_dyn
)

# Put binary for installation temporarily in build/src/python.
install(TARGETS spatialdyn
    LIBRARY DESTINATION "${PROJECT_BINARY_DIR}"
)
install(TARGETS spatialeigen
    LIBRARY DESTINATION "${PROJECT_BINARY_DIR}"
)
