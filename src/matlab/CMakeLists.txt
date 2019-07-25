############################################################
# CMakeLists for spatial_dyn Matlab wrappers
#
# Copyright 2019. All Rights Reserved.
#
# Created: April 25, 2019
# Authors: Toki Migimatsu
############################################################

find_package(Matlab REQUIRED)

function(add_matlab_function FILE_NAME)
    set(FUNCTION_NAME spatial_dyn_${FILE_NAME})
    matlab_add_mex(NAME ${FUNCTION_NAME}
        SRC
            ${LIB_SOURCE_DIR}/matlab/${ARGV1}${FILE_NAME}.cc
            ${SPATIAL_DYN_SRC}
        LINK_TO
            spatial_dyn::spatial_dyn
            ${Matlab_ROOT_DIR}/extern/bin/glnxa64/libMatlabEngine.so
            ${Matlab_ROOT_DIR}/extern/bin/glnxa64/libMatlabDataArray.so
    )

    target_include_directories(${FUNCTION_NAME}
        PUBLIC
            $<INSTALL_INTERFACE:include>
            $<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>
    )

    set_target_properties(${FUNCTION_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/matlab"
    )
endfunction()

add_matlab_function(delete_articulated_body)
add_matlab_function(load_model)
add_matlab_function(inertia)
add_matlab_function(centrifugal_coriolis)
add_matlab_function(gravity)
add_matlab_function(discrete_inverse_dynamics)
add_matlab_function(position)
add_matlab_function(jacobian)
add_matlab_function(jacobian_dynamic_inverse)
add_matlab_function(hessian)
add_matlab_function(inverse_kinematics)
add_matlab_function(inverse_dynamics)
add_matlab_function(inverse_dynamics_position_derivative)
add_matlab_function(inverse_dynamics_velocity_derivative)
