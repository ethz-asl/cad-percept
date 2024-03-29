cmake_minimum_required(VERSION 2.8.3)
project(cgal_visualizations)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++17 -Wall)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if (rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
    qt4_wrap_cpp(MOC_FILES
            ${INCLUDE_FILES_QT}
            )
else ()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
    QT5_WRAP_CPP(MOC_FILES
            ${INCLUDE_FILES_QT}
            )
endif ()

## Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

set(HEADER_FILES
        include/cgal_visualizations/mesh_visual.h
        include/cgal_visualizations/mesh_display.h
        include/cgal_visualizations/mesh_display_instantiations.h
        )

set(SRC_FILES
        src/mesh_visual.cc
        src/mesh_display.cc
        )

cs_add_library(${PROJECT_NAME}
        ${SRC_FILES}
        ${HEADER_FILES}
        ${MOC_FILES}
        )

target_link_libraries(${PROJECT_NAME}
        ${QT_LIBRARIES}
        )

##########
# EXPORT #
##########
cs_install()
cs_export()
