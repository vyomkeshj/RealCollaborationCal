cmake_minimum_required(VERSION 3.13)
project(RealCollaborationCal)

set(CMAKE_CXX_STANDARD 11)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files
set (CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES headers/modbus.h headers/CameraTCPOrientation.h classes/CameraTCPOrientation.cpp
        headers/modbus_exception.h classes/modbus.cpp headers/RealsenseManager.h
        headers/RealsensePoseEstimation.h
        headers/PCLvisualiser.h headers/RealsenseDeviceProvider.h )


add_executable(RealCollaborationCal UiMain.cpp classes/VisionsOfJohanna.cpp
        classes/CameraTCPOrientation.cpp classes/modbus.cpp
        classes/CameraTagInBaseCoordinates.cpp
        headers/CameraTagInBaseCoordinates.h headers/visionsofjohanna.hpp
        headers/realsenseManager.h headers/RealsenseDeviceProvider.h)

find_package( VTK 8.1 REQUIRED)

find_package(realsense2 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package( PCL 1.9.1 REQUIRED)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)
find_package(Qt5Core REQUIRED)

link_directories(${QT_LIBRARY_DIRS})
TARGET_LINK_LIBRARIES(RealCollaborationCal ${QT_LIBRARIES})


include_directories(include ${realsense_INCLUDE_DIR})
target_link_libraries(RealCollaborationCal ${realsense2_LIBRARY})
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(RealCollaborationCal ${OpenCV_LIBS})

target_include_directories(RealCollaborationCal PRIVATE headers qt_ui)
target_sources(RealCollaborationCal PRIVATE classes)

if( PCL_FOUND )
    # Additional Include Directories
    include_directories( ${PCL_INCLUDE_DIRS} )

    # Preprocessor Definitions
    add_definitions( ${PCL_DEFINITIONS} )

    # Additional Library Directories
    link_directories( ${PCL_LIBRARY_DIRS} )
    message(${PCL_LIBRARIES})
    target_link_libraries(RealCollaborationCal ${PCL_LIBRARIES} )


endif()

target_link_libraries(RealCollaborationCal Qt5::Widgets )
