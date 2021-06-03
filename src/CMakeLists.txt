############################################################
# CMakeLists for the spatial_dyn library.
#
# Copyright 2018. All Rights Reserved.
#
# Created: June 7, 2018
# Authors: Toki Migimatsu
############################################################

set(LIB_NAME ${PROJECT_NAME})

# Create the library.
add_library(${LIB_NAME} SHARED)
add_library(${LIB_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

target_compile_features(${LIB_NAME} PUBLIC cxx_std_14)
set_target_properties(${LIB_NAME} PROPERTIES CXX_EXTENSIONS OFF)

# Enable clang tidy checks.
if(${LIB_CMAKE_NAME}_CLANG_TIDY)
    target_enable_clang_tidy(${LIB_NAME})
endif()

# Set sources.
target_sources(${LIB_NAME}
  PRIVATE
    algorithms/discrete_dynamics.cc
    algorithms/dynamics_derivatives.cc
    algorithms/forward_dynamics.cc
    algorithms/forward_kinematics.cc
    algorithms/inverse_dynamics.cc
    algorithms/inverse_kinematics.cc
    algorithms/opspace_dynamics.cc
    algorithms/opspace_dynamics_aba.cc
    algorithms/simulation.cc
    structs/articulated_body.cc
    structs/graphics.cc
    structs/joint.cc
    structs/options.cc
    structs/rigid_body.cc
    parsers/json.cc
    parsers/urdf.cc
    parsers/yaml.cc
)

# Set include directories.
set(LIB_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
target_include_directories(${LIB_NAME}
  PUBLIC
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
    "$<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>"
  PRIVATE
    "${LIB_INCLUDE_DIR}/spatial_dyn"
)

# Link library dependencies.
lib_add_subdirectory(ctrl_utils)
ctrl_utils_add_subdirectory(Eigen3)
ctrl_utils_add_subdirectory(nlohmann_json)
ctrl_utils_add_subdirectory(yaml-cpp)
lib_add_subdirectory(tinyxml2)
target_link_libraries(${LIB_NAME}
  PUBLIC
    ctrl_utils::ctrl_utils
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
  PRIVATE
    tinyxml2::tinyxml2
)

# Build python wrapper.
if(${LIB_CMAKE_NAME}_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# Install the library.
install(TARGETS ${LIB_NAME}
    EXPORT ${LIB_NAME}Targets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

# Install dependencies (only works for cmake>=3.13).
if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.13.0")
    lib_install_subdirectory_yaml_cpp()
    lib_install_subdirectory_tinyxml2()
endif()

# Install the headers.
install(DIRECTORY "${LIB_INCLUDE_DIR}/${LIB_NAME}"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

# Create LibConfig.cmake
set(LIB_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${LIB_NAME}")
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${LIB_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${LIB_NAME}Config.cmake"
    INSTALL_DESTINATION "${LIB_CONFIG_INSTALL_DIR}"
)

# Create LibConfigVersion.cmake.
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/${LIB_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion
)

# Install LibConfig.cmake and LibConfigVersion.cmake.
install(FILES
    "${PROJECT_BINARY_DIR}/${LIB_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${LIB_NAME}ConfigVersion.cmake"
    DESTINATION ${LIB_CONFIG_INSTALL_DIR}
)

# Export targets to LibTargets.cmake.
install(EXPORT ${LIB_NAME}Targets
    DESTINATION ${LIB_CONFIG_INSTALL_DIR}
    NAMESPACE ${LIB_NAME}::
)

# Export targets to LibTargets.cmake for the build tree.
export(TARGETS ${LIB_NAME}
    NAMESPACE ${LIB_NAME}::
    FILE ${LIB_NAME}Targets.cmake
)
