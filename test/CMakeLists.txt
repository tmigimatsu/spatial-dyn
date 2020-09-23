############################################################
# CMakeLists for spatial_dyn tests
#
# Copyright 2018. All Rights Reserved.
#
# Created: September 11, 2018
# Authors: Toki Migimatsu
############################################################

# Download rbdl
execute_process(COMMAND git clone https://github.com/rbdl/rbdl.git ${LIB_EXTERNAL_DIR}/rbdl.git -b v2.6.0 --single-branch)

# Link Eigen to rbdl
get_property(EIGEN3_INCLUDE_DIR TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
add_subdirectory(${LIB_EXTERNAL_DIR}/rbdl.git ${CMAKE_BINARY_DIR}/rbdl)

# Create tests executable
add_executable(${CMAKE_PROJECT_NAME}_tests ${CMAKE_CURRENT_SOURCE_DIR}/tests.cc)

add_subdirectory(${ctrl_utils_EXTERNAL_DIR}/Catch2 ${CMAKE_CURRENT_BINARY_DIR}/Catch2)

target_link_libraries(${CMAKE_PROJECT_NAME}_tests PRIVATE
    spatial_dyn::spatial_dyn
    rbdl
    Catch2::Catch2
)

target_include_directories(${CMAKE_PROJECT_NAME}_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/rbdl.git/include
    ${CMAKE_BINARY_DIR}/rbdl/include)

add_test(${CMAKE_PROJECT_NAME}_tests ${CMAKE_PROJECT_NAME}_tests)
