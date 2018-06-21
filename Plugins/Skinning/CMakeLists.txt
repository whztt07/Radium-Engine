project(SkinningPlugin)

message("${PROJECT_NAME} output path : ${RADIUM_PLUGIN_OUTPUT_PATH}")

set(skinning_target Skinning)

# ------------------------------------------------------------------------------
# find dependencies
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(Qt5_LIBRARIES Qt5::Core Qt5::Widgets)

# ------------------------------------------------------------------------------
# get sources
file(GLOB_RECURSE sources src/*.cpp)
file(GLOB_RECURSE headers src/*.hpp)
file(GLOB_RECURSE inlines src/*.inl)

# ------------------------------------------------------------------------------
# configure
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# ------------------------------------------------------------------------------
# add library
set(skinning_libs
    ${Qt5_LIBRARIES}
    ${RADIUM_LIBRARIES}
    ${GLBINDING_LIBRARIES} # Radium dep
)

add_library(${skinning_target} SHARED ${sources} ${headers} ${inlines})

target_include_directories(${skinning_target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src> # Radium
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # MOC
    $<INSTALL_INTERFACE:include/Radium>
    $<INSTALL_INTERFACE:include/Radium/Plugins>
)

target_compile_definitions(${skinning_target} PRIVATE "-DSkinning_EXPORTS")
set_target_properties(${skinning_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${skinning_target} ${skinning_libs})

message(STATUS "${skinning_target} : dependencies are ${RADIUM_LIBRARIES}")
add_dependencies(${skinning_target} ${RADIUM_LIBRARIES})

# ------------------------------------------------------------------------------
# prepare install
install(TARGETS ${skinning_target}
    EXPORT   RadiumPluginTargets
    LIBRARY  DESTINATION "${lib_install_dir}"
    ARCHIVE  DESTINATION "${lib_install_dir}"
    RUNTIME  DESTINATION "${bin_install_dir}"
    INCLUDES DESTINATION "${include_install_dir}"
)

