# Build Simplifier

set(app_target subdivide)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(OpenGL     REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets OpenGL REQUIRED)

set(Qt5_LIBRARIES
    Qt5::Core
    Qt5::Widgets
    Qt5::OpenGL
)

set(app_libs
    ${RADIUM_LIBRARIES}    # Radium libs
    glbinding_lib # Radium dep
    assimp_lib    # Radium dep
    openmesh_lib  # Radium dep
    ${Qt5_LIBRARIES}       # the Qt beast
)

file(GLOB_RECURSE app_sources *.cpp)
file(GLOB_RECURSE app_headers *.h *.hpp)
file(GLOB_RECURSE app_inlines *.inl)
file(GLOB_RECURSE app_uis     *.ui)
qt5_wrap_ui(app_uis_moc ${app_uis})

file(GLOB resources ${CMAKE_CURRENT_SOURCE_DIR}/../../*.qrc)


include_directories(
    .
    ${RADIUM_INCLUDE_DIRS}
    ${PATATE_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR} # Moc
)

add_executable(
    ${app_target}
    ${app_sources}
    ${app_headers}
    ${app_inlines}
    ${app_uis_moc}
    ${resources}
)

target_link_libraries(
    ${app_target}
    ${app_libs}
)

add_dependencies(${app_target}
    radiumEngine radiumGuiBase
    radiumCore radiumIO
    radium_assets radium_configs
)

if(MSVC)
    #This will work from CMake 3.6
    set_property(DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${app_target})
    set_property(TARGET ${app_target} PROPERTY IMPORTED_LOCATION "${RADIUM_BINARY_OUTPUT_PATH}")
endif(MSVC)

install(TARGETS ${app_target} RUNTIME
    DESTINATION "${CMAKE_INSTALL_PREFIX}/bin/Radium"
)

if(APPLE)
    include(DeployQt)
    macdeployqt(${app_target})
endif()
