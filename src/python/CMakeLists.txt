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

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/python)
set(PYBIND11_LIB_DIR ${SPATIAL_DYN_LIB_DIR}/pybind11.git)
add_subdirectory(${PYBIND11_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/pybind11)

pybind11_add_module(spatialdyn spatial_dyn.cc eigen.cc ${SPATIAL_DYN_SRC})
target_link_libraries(spatialdyn
    PRIVATE ${SPATIAL_DYN_EXTERNAL_LIBS}
    PUBLIC ${SPATIAL_DYN_PUBLIC_LIBS})

pybind11_add_module(eigen eigen.cc ${SPATIAL_DYN_SRC})
target_link_libraries(eigen
    PRIVATE ${SPATIAL_DYN_EXTERNAL_LIBS}
    PUBLIC ${SPATIAL_DYN_PUBLIC_LIBS})

# pybind11_add_module(spatial-dyn-opspace spatial_dyn_opspace.cc ${SPATIAL_DYN_SRC})
# target_link_libraries(spatial-dyn-opspace
#     PRIVATE ${SPATIAL_DYN_EXTERNAL_LIBS}
#     PUBLIC ${SPATIAL_DYN_PUBLIC_LIBS})

# pybind11_add_module(spatial-dyn-urdf spatial_dyn_urdf.cc ${SPATIAL_DYN_SRC})
# target_link_libraries(spatial-dyn-urdf
#     PRIVATE ${SPATIAL_DYN_EXTERNAL_LIBS}
#     PUBLIC ${SPATIAL_DYN_PUBLIC_LIBS})
