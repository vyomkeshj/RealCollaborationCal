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
set(CMAKE_CXX_FLAGS "-lncurses")

set(SOURCES headers/modbus.h headers/RobotJointAngles.h classes/RobotJointAngles.cpp
        headers/modbus_exception.h classes/modbus.cpp headers/RealsenseManager.h
        headers/RealsensePoseEstimation.h
        headers/PhoXiConversions.h
        headers/PhoXiException.h
        headers/PhoXiDeviceInformation.h
        headers/PhoXiInterface.h
        headers/Types.h
        )




add_executable(RealCollaborationCal UiMain.cpp classes/VisionsOfJohanna.cpp
        classes/RobotJointAngles.cpp classes/modbus.cpp
        headers/CameraTagInBaseCoordinates.h headers/visionsofjohanna.hpp
        headers/RealsenseManager.h headers/RealsenseDeviceProvider.h
        headers/EigenFileExtension.h
        ur3-livemodel/Artifact.cpp

        headers/PhoXiConversions.h
        headers/PhoXiException.h
        headers/PhoXiDeviceInformation.h
        headers/PhoXiInterface.h
        headers/Types.h

        ur3-livemodel/headers/Artifact.h ur3-livemodel/Joint.cpp ur3-livemodel/headers/Joint.h ur3-livemodel/RobotModel.cpp
        ur3-livemodel/headers/RobotModel.h ur3-livemodel/headers/RobotPart.h ur3-livemodel/RobotModelImpl.cpp
        ur3-livemodel/headers/RobotModelImpl.h ur3-livemodel/RobotPart.cpp ur3-livemodel/CollisionArtifact.cpp
        ur3-livemodel/headers/CollisionArtifact.h classes/PhoXiInterface.cpp
        headers/BehindVisionsOfJohanna.h classes/BehindVisionsOfJohanna.cpp )

#[[

add_executable(RealCollaborationCal main.cpp
        classes/RobotJointAngles.cpp classes/modbus.cpp
        classes/CameraTagInBaseCoordinates.cpp
        headers/CameraTagInBaseCoordinates.h
        headers/RealsenseManager.h headers/RealsenseDeviceProvider.h headers/EigenFileExtension.h)

]]

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(VTK 8.1 REQUIRED)

find_package(realsense2 REQUIRED)

set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost REQUIRED)

find_package(OpenCV 4 REQUIRED)
find_package(PCL 1.9.1 REQUIRED)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)
find_package(Qt5Core REQUIRED)

link_directories(${QT_LIBRARY_DIRS})
TARGET_LINK_LIBRARIES(RealCollaborationCal ${QT_LIBRARIES})

include_directories(include ${realsense_INCLUDE_DIR}
        ${PHOXI_INCLUDE_DIRS})
include(${VTK_USE_FILE})

add_library(
        ${PROJECT_NAME}_PhoXi_Interface
        classes/PhoXiInterface.cpp
)


target_link_libraries(RealCollaborationCal ${realsense2_LIBRARY})
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(RealCollaborationCal ${OpenCV_LIBS})

target_include_directories(RealCollaborationCal PRIVATE headers perf_upg)
target_sources(RealCollaborationCal PRIVATE classes)

target_link_libraries(RealCollaborationCal
        ${PHOLOCALIZATION_LIBRARIES}
        ${PHOXI_LIBRARY}  )


target_link_libraries(RealCollaborationCal ${Boost_LIBRARIES})
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
target_link_libraries(RealCollaborationCal ${VTK_LIBRARIES})

target_link_libraries(RealCollaborationCal Qt5::Widgets Qt5::Core)


