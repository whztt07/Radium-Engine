project(SkinningPlugin)

cmake_minimum_required(VERSION 2.8.11)
if (APPLE)
    cmake_policy(SET CMP0042 NEW)
    # RPATH settings on macOS do not affect install_name.
    # Run "cmake --help-policy CMP0068" for policy details.
    cmake_policy(SET CMP0068 NEW)
endif(APPLE)

message("${PROJECT_NAME} output path : ${RADIUM_PLUGIN_OUTPUT_PATH}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RADIUM_PLUGIN_OUTPUT_PATH})
set(EXECUTABLE_OUTPUT_PATH         ${RADIUM_PLUGIN_OUTPUT_PATH})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${RADIUM_PLUGIN_OUTPUT_PATH})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${RADIUM_PLUGIN_OUTPUT_PATH})

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

file(GLOB_RECURSE sources src/*.cpp)
file(GLOB_RECURSE headers src/*.hpp)
file(GLOB_RECURSE inlines src/*.inl)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories(
    src/
    ${CMAKE_CURRENT_BINARY_DIR} # MOC
    ${RADIUM_INCLUDE_DIRS}
)

add_library(Skinning SHARED ${sources} ${headers} ${inlines})
message(INFO " Skinning : dependencies are ${RADIUM_LIBRARIES}")
add_dependencies(Skinning ${RADIUM_LIBRARIES})

target_compile_definitions(Skinning PRIVATE "-DSkinning_EXPORTS")

set(Qt5_LIBRARIES Qt5::Core Qt5::Widgets)
target_link_libraries(Skinning
    ${Qt5_LIBRARIES}
    ${RADIUM_LIBRARIES}
    ${GLBINDING_LIBRARIES} # Radium dep
)

install(TARGETS Skinning
    EXPORT SkinningTargets
    LIBRARY  DESTINATION "${CMAKE_INSTALL_PREFIX}/bin/Radium/Plugins"
    ARCHIVE  DESTINATION "${CMAKE_INSTALL_PREFIX}/bin/Radium/Plugins"
    RUNTIME  DESTINATION "${CMAKE_ISNTALL_PREFIX}/bin/Radium"
    INCLUDES DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Radium"
)
