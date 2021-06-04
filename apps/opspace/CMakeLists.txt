############################################################
# CMakeLists for the exmaple Operational Space controller
#
# Copyright 2019. All Rights Reserved.
#
# Created: January 29, 2019
# Authors: Toki Migimatsu
############################################################

# Require 3.6 to support pkg_check_modules IMPORTED_TARGET
# cmake_minimum_required(VERSION 3.6)

# # Define project
# project(spatial_dyn_opspace VERSION 1.0.0 LANGUAGES CXX)

# # Set CMake flags
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
#     set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
# endif()

# # Define directories
# set(SPATIAL_DYN_OPSPACE_BIN spatial_dyn_opspace)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(SPATIAL_DYN_OPSPACE_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# # Define binary
# add_executable(${SPATIAL_DYN_OPSPACE_BIN})

# # Set compiler options
# target_compile_options(${SPATIAL_DYN_OPSPACE_BIN} PRIVATE
#     $<$<CXX_COMPILER_ID:GNU>:-Wall>
#     $<$<CONFIG:RELEASE>:-O3>
# )

# # Link dependencies
# find_package(spatial_dyn 1.0.0 REQUIRED)

# target_link_libraries(${SPATIAL_DYN_OPSPACE_BIN}
#     PRIVATE
#         spatial_dyn::spatial_dyn
# )

# # Attach sources
# target_sources(${SPATIAL_DYN_OPSPACE_BIN}
#     PRIVATE
#         ${SPATIAL_DYN_OPSPACE_SRC_DIR}/main.cc
# )
