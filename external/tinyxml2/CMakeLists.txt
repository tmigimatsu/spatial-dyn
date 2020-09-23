############################################################
# CMakeLists for spatial_dyn
#
# Copyright 2020. All Rights Reserved.
#
# Created: September 23, 2020
# Authors: Toki Migimatsu
############################################################

if(NOT TARGET tinyxml2::tinyxml2)
    find_package(tinyxml2 8.0.0 QUIET)

    if(NOT tinyxml2_FOUND)
        set(PREV_BUILD_TESTING ${BUILD_TESTING})
        set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

        init_git_submodule(tinyxml2.git)
        add_subdirectory(tinyxml2.git ${CMAKE_BINARY_DIR}/tinyxml2)

        set(BUILD_TESTING ${PREV_BUILD_TESTING} CACHE BOOL "" FORCE)
    endif()

    add_library(tinyxml2::tinyxml2 INTERFACE IMPORTED GLOBAL)
    target_link_libraries(tinyxml2::tinyxml2 INTERFACE tinyxml2)
endif()
