############################################################
# CMakeLists for spatial_dyn tests
#
# Copyright 2018. All Rights Reserved.
#
# Created: September 11, 2018
# Authors: Toki Migimatsu
############################################################

add_executable(${CMAKE_PROJECT_NAME}_tests ${CMAKE_CURRENT_SOURCE_DIR}/tests.cc)

find_package(PkgConfig)
pkg_check_modules(rbdl QUIET IMPORTED_TARGET RBDL>=2.5)
if(NOT RBDL_FOUND)
    download_package(rbdl 2.6.0)
    pkg_check_modules(rbdl REQUIRED IMPORTED_TARGET
        ${CMAKE_BINARY_DIR}/rbdl/build/rbdl.pc>=2.5)
endif()

find_package(Catch2 2.4.0 QUIET)
if(NOT Catch2_FOUND)
    download_package(Catch2 2.4.0)
    add_library(Catch2 INTERFACE)
    target_include_directories(Catch2 INTERFACE
        $<BUILD_INTERFACE:${LIB_EXTERNAL_DIR}/Catch2.git/single_include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    add_library(Catch2::Catch2 ALIAS Catch2)
endif()

target_link_libraries(${CMAKE_PROJECT_NAME}_tests PRIVATE
    spatial_dyn::spatial_dyn
    PkgConfig::rbdl
    Catch2::Catch2
)
add_test(${CMAKE_PROJECT_NAME}_tests ${CMAKE_PROJECT_NAME}_tests)
