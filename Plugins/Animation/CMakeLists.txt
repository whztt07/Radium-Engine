project(AnimationPlugin)

message("${PROJECT_NAME} output path : ${RADIUM_PLUGIN_OUTPUT_PATH}")

set(animation_target Animation)

# ------------------------------------------------------------------------------
# find dependencies
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(Qt5_LIBRARIES Qt5::Core Qt5::Widgets)

# ------------------------------------------------------------------------------
# get sources
file(GLOB_RECURSE sources src/*.cpp)
file(GLOB_RECURSE headers src/*.hpp src/*.h)
file(GLOB_RECURSE inlines src/*.inl)

file(GLOB_RECURSE app_uis  src/*.ui)
qt5_wrap_ui(app_uis_moc ${app_uis})

# ------------------------------------------------------------------------------
# configure
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ------------------------------------------------------------------------------
# add library
set(animation_libs
    ${Qt5_LIBRARIES}
    ${RADIUM_LIBRARIES}
    ${GLBINDING_LIBRARIES} # Radium dep
)

add_library(${animation_target} SHARED ${sources} ${headers} ${inlines} ${app_uis})

target_include_directories(${animation_target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src> # Radium
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # MOC
    $<INSTALL_INTERFACE:include/Radium>
    $<INSTALL_INTERFACE:include/Radium/Plugins>
)

target_compile_definitions(${animation_target} PRIVATE "-DAnimation_EXPORTS")
set_target_properties(${animation_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${animation_target} ${animation_libs})

message(STATUS "${animation_target} : dependencies are ${RADIUM_LIBRARIES}")
add_dependencies(${animation_target} ${RADIUM_LIBRARIES})

# ------------------------------------------------------------------------------
# prepare install
install(TARGETS ${animation_target}
    EXPORT   RadiumPluginTargets
    LIBRARY  DESTINATION "${lib_install_dir}"
    ARCHIVE  DESTINATION "${lib_install_dir}"
    RUNTIME  DESTINATION "${bin_install_dir}"
    INCLUDES DESTINATION "${include_install_dir}"
)
