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
    message(${PCL_LIBRARIES})


    # Additional Dependencies
    target_link_libraries( RealCollaborationCal  pcl_kdtree pcl_octree pcl_search
            pcl_sample_consensus pcl_filters pcl_io pcl_features pcl_ml pcl_segmentation
            pcl_visualization pcl_surface pcl_registration pcl_keypoints pcl_tracking
            pcl_recognition pcl_stereo pcl_outofcore pcl_people
            /usr/lib/x86_64-linux-gnu/libboost_system.so
            /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
            /usr/lib/x86_64-linux-gnu/libboost_thread.so -lpthread
            /usr/lib/x86_64-linux-gnu/libboost_date_time.so
            /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
            /usr/lib/x86_64-linux-gnu/libboost_serialization.so
            /usr/lib/x86_64-linux-gnu/libboost_chrono.so
            /usr/lib/x86_64-linux-gnu/libboost_atomic.so
            /usr/lib/x86_64-linux-gnu/libboost_regex.so flannflann_cpp VTK::WrappingTools VTK::ViewsInfovis VTK::CommonCore
            VTK::CommonDataModel VTK::CommonExecutionModel VTK::InteractionStyle VTK::RenderingContext2D
            VTK::ViewsCore VTK::ChartsCore VTK::CommonColor VTK::CommonTransforms VTK::FiltersCore
            VTK::FiltersExtraction VTK::FiltersGeneral VTK::FiltersGeometry VTK::FiltersImaging VTK::FiltersModeling
            VTK::FiltersSources VTK::FiltersStatistics VTK::ImagingGeneral VTK::InfovisCore VTK::InfovisLayout
            VTK::InteractionWidgets VTK::RenderingAnnotation VTK::RenderingCore VTK::RenderingLabel
            VTK::IOImage VTK::IOInfovis VTK::IOSQL VTK::IOXML VTK::RenderingContextOpenGL2
            VTK::TestingRendering VTK::ViewsContext2D VTK::Java VTK::RenderingOpenGL2 VTK::TestingCore
            VTK::CommonSystem VTK::ImagingCore VTK::vtksys VTK::PythonContext2D VTK::WrappingPythonCore
            VTK::RenderingVolumeOpenGL2 VTK::ImagingMath VTK::RenderingVolume VTK::CommonMath VTK::glew
            VTK::opengl VTK::IOLegacy VTK::ImagingSources VTK::PythonInterpreter VTK::Python
            VTK::CommonMisc VTK::RenderingFreeType VTK::octree VTK::RenderingLOD VTK::RenderingImage
            VTK::IOVeraOut VTK::hdf5 VTK::IOTecplotTable VTK::IOCore VTK::IOSegY VTK::IOParallelXML
            VTK::ParallelCore VTK::IOPLY VTK::IOOggTheora VTK::IOMovie VTK::theora VTK::ogg
            VTK::IOMotionFX VTK::IOGeometry VTK::pegtl VTK::IOParallel VTK::IONetCDF
            VTK::jsoncpp VTK::FiltersParallel VTK::exodusII VTK::netcdf VTK::InteractionImage
            VTK::RenderingParallel VTK::IOMINC VTK::FiltersHybrid VTK::IOLSDyna VTK::IOXMLParser
            VTK::libxml2 VTK::zlib VTK::IOImport VTK::IOAMR VTK::IOVideo VTK::IOExportPDF
            VTK::IOExport VTK::libharu VTK::IOExportGL2PS VTK::RenderingGL2PSOpenGL2
            VTK::gl2ps VTK::ImagingHybrid VTK::png VTK::IOExodus VTK::IOEnSight VTK::IOCityGML
            VTK::pugixml VTK::IOAsynchronous VTK::FiltersAMR VTK::ImagingColor VTK::ImagingStencil
            VTK::CommonComputationalGeometry VTK::ImagingStatistics VTK::ImagingMorphological
            VTK::sqlite VTK::TestingIOSQL VTK::GeovisCore VTK::libproj VTK::FiltersFlowPaths
            VTK::FiltersProgrammable VTK::FiltersSelection VTK::FiltersTopology
            VTK::FiltersTexture VTK::FiltersSMP VTK::FiltersPython VTK::FiltersPoints
            VTK::FiltersVerdict VTK::verdict VTK::FiltersParallelImaging VTK::FiltersHyperTree
            VTK::FiltersGeneric VTK::TestingGenericBridge VTK::IOParallelExodus VTK::DICOMParser
            VTK::jpeg VTK:metaio VTK::tiff VTK::DomainsChemistry VTK::DomainsChemistryOpenGL2
            VTK::expat VTK::doubleconversion VTK::lz4 VTK::lzma VTK::utf8 VTK::ImagingFourier
            VTK::eigen VTK::freetype VTK::RenderingMatplotlib VTK::kwiml
            )
endif()

