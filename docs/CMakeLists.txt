############################################################
# CMakeLists for spatialdyn docs.
#
# Copyright 2018. All Rights Reserved.
#
# Created: September 11, 2018
# Authors: Toki Migimatsu
############################################################

message(STATUS "Configuring ${PROJECT_NAME} documentation.")

add_custom_target(spatialdyn_docs ALL
    COMMAND pipenv run make -C docs clean && pipenv run make -C docs
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
add_dependencies(spatialdyn_docs spatialdyn)
