############################################################
# Utility scripts for downloading external packages
#
# Copyright 2020. All Rights Reserved.
#
# Created: June 5, 2020
# Authors: Toki Migimatsu
############################################################

function(init_git_submodule GIT_SUBMODULE)
    set(RECURSIVE "")
    if(DEFINED ARGV1)
        if (${ARGV1})
            set(RECURSIVE "--recursive")
        endif()
    endif()

    # Update submodule
    find_package(Git REQUIRED)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} submodule update --init ${RECURSIVE} ${GIT_SUBMODULE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE git_submodule_result
    )
    if(NOT git_submodule_result EQUAL "0")
        message(FATAL_ERROR "${GIT_EXECUTABLE} submodule update --init ${RECURSIVE} ${GIT_SUBMODULE} failed with error:\n ${git_submodule_result}")
    endif()
endfunction()
