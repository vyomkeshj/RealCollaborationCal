cmake_minimum_required(VERSION 3.13)
project(RealCollaborationCal)

set(CMAKE_CXX_STANDARD 17)


set(SOURCES headers/modbus.h headers/CameraTCPOrientation.h classes/CameraTCPOrientation.cpp
        headers/modbus_exception.h classes/modbus.cpp headers/RealsenseManager.h
        headers/RealsensePoseEstimation.h)


add_executable(RealCollaborationCal main.cpp classes/CameraTCPOrientation.cpp classes/modbus.cpp classes/CameraTagInBaseCoordinates.cpp headers/CameraTagInBaseCoordinates.h classes/PCLvisualiser.cpp headers/PCLvisualiser.h)

find_package(realsense2 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package( PCL 1.9.1 REQUIRED)

include_directories(include ${realsense_INCLUDE_DIR})
target_link_libraries(RealCollaborationCal ${realsense2_LIBRARY})
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(RealCollaborationCal ${OpenCV_LIBS})

target_include_directories(RealCollaborationCal PRIVATE headers)
target_sources(RealCollaborationCal PRIVATE classes)


if( PCL_FOUND )
    # Additional Include Directories
    include_directories( ${PCL_INCLUDE_DIRS} )

    # Preprocessor Definitions
    add_definitions( ${PCL_DEFINITIONS} )

    # Additional Library Directories
    link_directories( ${PCL_LIBRARY_DIRS} )

    # Additional Dependencies
    target_link_libraries( RealCollaborationCal ${PCL_LIBRARIES} )
endif()

