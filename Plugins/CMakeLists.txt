message(STATUS "Building plugins with radium libs : ${RADIUM_LIBRARIES} ")

# ------------------------------------------------------------------------------
# configure
cmake_minimum_required(VERSION 2.8.11)
if (APPLE)
    cmake_policy(SET CMP0042 NEW)
    # RPATH settings on macOS do not affect install_name.
    # Run "cmake --help-policy CMP0068" for policy details.
    cmake_policy(SET CMP0068 NEW)
endif(APPLE)
cmake_policy(SET CMP0048 OLD) # plugins project are versionned with Radium's version

set(CMAKE_DEBUG_POSTFIX "")


# ------------------------------------------------------------------------------
# set build paths
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RADIUM_PLUGIN_OUTPUT_PATH})
set(EXECUTABLE_OUTPUT_PATH         ${RADIUM_PLUGIN_OUTPUT_PATH})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${RADIUM_PLUGIN_OUTPUT_PATH})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${RADIUM_PLUGIN_OUTPUT_PATH})


# ------------------------------------------------------------------------------
# set install paths
set(config_install_dir "${CMAKE_INSTALL_PREFIX}/lib/Radium/cmake/")
set(include_install_dir "${CMAKE_INSTALL_PREFIX}/include/Radium/Plugins")
set(bin_install_dir "${CMAKE_INSTALL_PREFIX}/bin/Radium/Plugins")
set(lib_install_dir "${CMAKE_INSTALL_PREFIX}/bin/Radium/Plugins")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/Radium/Plugins")


# ------------------------------------------------------------------------------
# build plugins
add_subdirectory(FancyMesh)
add_subdirectory(Animation)
add_subdirectory(Skinning)
add_subdirectory(MeshFeatureTracking)
add_subdirectory(MeshPaint)


# ------------------------------------------------------------------------------
# install/package related stuff (from https://cmake.org/cmake/help/v3.10/manual/cmake-packages.7.html)

# FIXME(Florian): should these 4 replace the <lib>_API macros? (like RA_CORE_API or FM_PLUGIN_API)
#include(GenerateExportHeader)
#generate_export_header(${plugin_target})

# install targets
export(EXPORT RadiumPluginTargets
    FILE "${generated_dir}/RadiumPluginTargets.cmake"
    NAMESPACE Ra::
)
install(EXPORT RadiumPluginTargets
    FILE        RadiumPluginTargets.cmake
    NAMESPACE   Ra::
    DESTINATION "${config_install_dir}"
)

# install headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION "${include_install_dir}"
    FILES_MATCHING PATTERN "*.h"  PATTERN "*.hpp"  PATTERN "*.inl"
)

# build Config.cmake files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${generated_dir}/RadiumPluginConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file("${CMAKE_SOURCE_DIR}/cmake/RadiumPluginConfig.cmake.in"
    "${generated_dir}/RadiumPluginConfig.cmake"
    INSTALL_DESTINATION "${config_install_dir}"
)
install(FILES "${generated_dir}/RadiumPluginConfig.cmake"
              "${generated_dir}/RadiumPluginConfigVersion.cmake"
    DESTINATION "${config_install_dir}"
)

