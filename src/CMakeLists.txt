set(library_version "0.1.0")

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

####
# Find packages
####
find_package(Qt5 COMPONENTS Core Widgets OpenGL Xml REQUIRED)
find_package(OpenMesh REQUIRED)
find_package(Eigen3 REQUIRED)
if(RADIUM_ASSIMP_SUPPORT)
    find_package(Assimp REQUIRED)
    find_library(ASSIMP_LIBS NAMES ${ASSIMP_LIBRARIES} PATHS ${ASSIMP_LIBRARY_DIRS})
    set(ASSIMP_LIBRARIES ${ASSIMP_LIBS})
    unset(ASSIMP_LIBS)
endif(RADIUM_ASSIMP_SUPPORT)

if(Qt5Core_VERSION VERSION_LESS 5.5 OR
   Qt5Widgets_VERSION VERSION_LESS 5.5 OR
   Qt5OpenGL_VERSION VERSION_LESS 5.5)
    message(FATAL_ERROR "Qt5 or superior required (found ${Qt5Core_VERSION}).")
else ()
    message(STATUS "QT ${Qt5Core_VERSION} found." )
endif()

message(STATUS "Compiling Radium with 3rdparty include dirs :
    ${EIGEN3_INCLUDE_DIR} --
    ${GLBINDING_INCLUDE_DIR} --
    ${GLM_INCLUDE_DIR} --
    ${GLOBJECTS_INCLUDE_DIR} --
    ${OPENMESH_INCLUDE_DIRS}"
)


####
# Build Core
####
set(core_target radiumCore)

set(RADIUM_VERSION_FILE ${CMAKE_CURRENT_BINARY_DIR}/Version.cpp)
add_custom_command(
    OUTPUT ${RADIUM_VERSION_FILE}
    COMMAND ${CMAKE_COMMAND}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_CXX_COMPILER_VERSION=${CMAKE_CXX_COMPILER_VERSION}
        -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
        -DCMAKE_CXX_FLAGS_DEBUG=${CMAKE_CXX_FLAGS_DEBUG}
        -DCMAKE_CXX_FLAGS_RELEASE=${CMAKE_CXX_FLAGS_RELEASE}
        -DGIT_CHANGESET=${GIT_CHANGESET}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/CMakeGenerateVersion.txt
    COMMENT "Generate Radium Version file"
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/CMakeGenerateVersion.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    VERBATIM
)
add_custom_target(radiumVersionFile DEPENDS ${RADIUM_VERSION_FILE})

file(GLOB_RECURSE core_sources Core/*.cpp)
file(GLOB_RECURSE core_headers Core/*.h Core/*.hpp)
file(GLOB_RECURSE core_inlines Core/*.inl)

set(core_libs ${OPENMESH_LIBRARIES})

add_library(${core_target} SHARED
    ${core_sources}
    ${core_headers}
    ${core_inlines}
    ${RADIUM_VERSION_FILE} # This file is build-dependant and generated in the Bundle
)

target_include_directories(${core_target} PUBLIC
    "$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>"
    "$<BUILD_INTERFACE:${OPENMESH_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:include/Radium>"
)
target_compile_definitions(${core_target} PRIVATE "-DRA_CORE_EXPORTS")
set_target_properties(${core_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${core_target} ${core_libs})

add_dependencies(${core_target} radiumVersionFile)


####
# Build IO
####
set(io_target radiumIO)

file(GLOB io_sources IO/*.cpp)
file(GLOB io_headers IO/*.h IO/*.hpp)
file(GLOB io_inlines IO/*.inl)

set(io_libs ${core_target})

set(RADIUM_IO_IS_INTERFACE TRUE)
if(RADIUM_ASSIMP_SUPPORT)
    add_subdirectory(IO/AssimpLoader)
    set(io_sources ${io_sources} ${assimp_sources})
    set(io_headers ${io_headers} ${assimp_headers})
    set(io_inlines ${io_inlines} ${assimp_inlines})
    set(io_libs    ${io_libs}    ${ASSIMP_LIBRARIES})
endif(RADIUM_ASSIMP_SUPPORT)

if(RADIUM_TINYPLY_SUPPORT)
    add_subdirectory(IO/TinyPlyLoader)
    set(io_sources ${io_sources} ${tinyply_sources})
    set(io_headers ${io_headers} ${tinyply_headers})
    set(io_inlines ${io_inlines} ${tinyply_inlines})
endif(RADIUM_TINYPLY_SUPPORT)

if(RADIUM_IO_IS_INTERFACE)
    add_library(${io_target} INTERFACE)
    target_sources(${io_target} INTERFACE
        ${io_sources}
        ${io_headers}
        ${io_inlines}
    )
    target_compile_definitions(${io_target} INTERFACE "-DRA_IO_STATIC")
    target_link_libraries(${io_target} ${io_libs})
else(RADIUM_IO_IS_INTERFACE)
    add_library(${io_target} SHARED
        ${io_sources}
        ${io_headers}
        ${io_inlines}
    )
    target_compile_definitions(${io_target} PRIVATE "-DRA_IO_EXPORTS")
    target_link_libraries(${io_target} ${io_libs})
    set_target_properties(${io_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif(RADIUM_IO_IS_INTERFACE)

target_include_directories(${io_target} PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/Radium>
)

add_dependencies(${io_target} ${core_target})

if(RADIUM_ASSIMP_SUPPORT)
    target_include_directories(${io_target} PUBLIC
        $<BUILD_INTERFACE:${ASSIMP_INCLUDE_DIRS}>
    )
endif(RADIUM_ASSIMP_SUPPORT)

if(RADIUM_TINYPLY_SUPPORT)
    target_include_directories(${io_target} PUBLIC
        $<BUILD_INTERFACE:${TINYPLY_INCLUDE_DIR}>
    )
    add_dependencies(${io_target} tinyply)
endif(RADIUM_TINYPLY_SUPPORT)


####
# Build Engine
####
set(engine_target radiumEngine)

file(GLOB_RECURSE engine_sources Engine/*.cpp)
file(GLOB_RECURSE engine_headers Engine/*.h Engine/*.hpp)
file(GLOB_RECURSE engine_inlines Engine/*.inl)

file(GLOB_RECURSE engine_glsl ../Shaders/*.glsl)

set(engine_shaders ${engine_glsl})

set(engine_libs
    ${core_target}
    ${io_target}
    ${Qt5OpenGL_LIBRARIES}
    ${GLBINDING_LIBRARIES}
    ${GLOBJECTS_LIBRARIES}
)

add_library(${engine_target} SHARED
    ${engine_sources}
    ${engine_headers}
    ${engine_inlines}
    ${engine_shaders}
)

target_include_directories(${engine_target} PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${GLBINDING_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${GLOBJECTS_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${GLM_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${STB_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/Radium>
)
target_compile_definitions(${engine_target} PRIVATE "-DRA_ENGINE_EXPORTS")
set_target_properties(${engine_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${engine_target} ${engine_libs})

add_dependencies(${engine_target} ${io_target} glbinding_lib globjects_lib glm_lib stb_lib)


####
# Build GuiBase
####
set(guibase_target radiumGuiBase)

# Qt5 stuff
set(Qt5_LIBRARIES Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Xml)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE guibase_sources GuiBase/*.cpp)
file(GLOB_RECURSE guibase_headers GuiBase/*.h GuiBase/*.hpp)
file(GLOB_RECURSE guibase_inlines GuiBase/*.inl)

set(guibase_libs
    ${core_target}
    ${io_target}
    ${engine_target}
    ${Qt5_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLBINDING_LIBRARIES}
    ${GLOBJECTS_LIBRARIES}
)

add_library(${guibase_target} SHARED
    ${guibase_sources}
    ${guibase_headers}
    ${guibase_inlines}
)

target_include_directories(${guibase_target} PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${STB_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/Radium>
)
target_compile_definitions(${guibase_target} PRIVATE "-DRA_GUIBASE_EXPORTS")
set_target_properties(${guibase_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${guibase_target} ${guibase_libs})

add_dependencies(${guibase_target} ${engine_target})


#####
# update RADIUM_LIBRARIES
####"
set(RADIUM_LIBRARIES
    ${RADIUM_LIBRARIES}
    "${core_target}"
    "${engine_target}"
    "${io_target}"
    "${guibase_target}"
    PARENT_SCOPE
)


#####
# Build tests
#####
add_subdirectory(Tests)


#####
# install/package related stuff (from https://cmake.org/cmake/help/v3.10/manual/cmake-packages.7.html)
#####
# set paths
set(config_install_dir "${CMAKE_INSTALL_PREFIX}/lib/Radium/cmake/")
set(include_install_dir "${CMAKE_INSTALL_PREFIX}/include/Radium")
set(bin_install_dir "${CMAKE_INSTALL_PREFIX}/bin/Radium")
set(lib_install_dir "${CMAKE_INSTALL_PREFIX}/lib/Radium")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/Radium")

# FIXME(Florian): should these 4 replace the <lib>_API macros? (like RA_CORE_API or FM_PLUGIN_API)
#include(GenerateExportHeader)
#generate_export_header(${core_target})
#generate_export_header(${io_target})
#generate_export_header(${engine_target})
#generate_export_header(${guibase_target})
# then shall we split each lib for install so that the installed headers include the generated export headers?

# install targets
install(TARGETS ${core_target} ${io_target} ${engine_target} ${guibase_target}
    EXPORT   RadiumTargets
    LIBRARY  DESTINATION "${lib_install_dir}"
    ARCHIVE  DESTINATION "${lib_install_dir}"
    RUNTIME  DESTINATION "${bin_install_dir}"
    INCLUDES DESTINATION "${include_install_dir}"
)
export(EXPORT RadiumTargets
    FILE "${generated_dir}/RadiumTargets.cmake"
    NAMESPACE Ra::
)
install(EXPORT RadiumTargets
    FILE        RadiumTargets.cmake
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
    "${generated_dir}/RadiumConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file("${CMAKE_SOURCE_DIR}/cmake/RadiumConfig.cmake.in"
    "${generated_dir}/RadiumConfig.cmake"
    INSTALL_DESTINATION "${config_install_dir}"
)
install(FILES "${generated_dir}/RadiumConfig.cmake"
              "${generated_dir}/RadiumConfigVersion.cmake"
    DESTINATION "${config_install_dir}"
)
