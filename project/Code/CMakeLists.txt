cmake_minimum_required(VERSION 3.5)
cmake_minimum_required(VERSION 3.5)

project(Robots VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_PREFIX_PATH "C:\\Qt\\6.5.2\\mingw_64\\lib\\cmake")
find_package(QT NAMES Qt6 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

set(COMMON_SOURCES
        State.h
        Robot.h
        Particle.h
        Position.h
        Types.h
        json.hpp
        json_util.h
        Timeline.h
        StateStream.h
        StateStream.cpp)

set(PROJECT_SOURCES
        main.cpp
        robotsui.cpp
        robotsui.h
        robotsui.ui
        robotwidget.h
        robotwidget.cpp
        worldtoscreenconverter.h
        worldtoscreenconverter.cpp)

set(PROJECT_SOURCES_SETUP
        mainSetup.cpp
        Simu.h
        Simu.cpp)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(Robots
        MANUAL_FINALIZATION
        ${COMMON_SOURCES}
        ${PROJECT_SOURCES}
    )
    add_executable(Setup
        ${COMMON_SOURCES}
        ${PROJECT_SOURCES_SETUP}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET Robots APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(Robots SHARED
            ${COMMON_SOURCES}
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(Robots
             ${COMMON_SOURCES}
             ${PROJECT_SOURCES}
        )
        add_executable(Setup
             ${COMMON_SOURCES}
             ${PROJECT_SOURCES_SETUP}
        )
    endif()

endif()

target_link_libraries(Robots PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(Robots PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS Robots
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(Robots)
endif()
