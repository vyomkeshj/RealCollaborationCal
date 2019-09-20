//
// Created by controller on 1/11/18.
//

#ifndef PROJECT_PHOXIINTERFACE_H
#define PROJECT_PHOXIINTERFACE_H

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <headers/PhoXiException.h>
#include <headers/PhoXiDeviceInformation.h>
#include <pcl/point_cloud.h>
#include <PhoXi.h>

// PhoXi API version
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define PHOXI_API_VERSION STR(PHOXI_API_VERSION_MAJOR) "." STR(PHOXI_API_VERSION_MINOR) "." STR(PHOXI_API_VERSION_PATCH)
#if PHOXI_API_VERSION_MAJOR == 1
#if PHOXI_API_VERSION_MINOR == 1
#define PHOXI_API_v1_1
#elif PHOXI_API_VERSION_MINOR == 2
#define PHOXI_API_v1_2
#endif
#endif

//* PhoXiInterface
/**
 * Wrapper to PhoXi 3D Scanner api to make interface easier
 *
 */
namespace phoxi_camera {
    class PhoXiInterface {
    public:

        /**
        * Default constructor.
        */
        PhoXiInterface();

        /**
        * Return all PhoXi 3D Scanners ids connected on network with all informations about dcevice.
        *
        * \throw PhoXiControlNotRunning when PhoXi Control is not running
        */
        std::vector<PhoXiDeviceInformation> deviceList();

        /**
        * Return all PhoXi 3D Scanners ids connected on network.
        *
        * \note returned id can be passed to connectCamera method
        * \throw PhoXiControlNotRunning when PhoXi Control is not running
        */
        std::vector<std::string> cameraList();

        /**
        * Connect to camera.
        *
        * \param HWIdentification - identification number
        * \param mode - trigger mode to set after connection
        * \param startAcquisition if true Acquisition will be started
        * \throw PhoXiControlNotRunning when PhoXi Control is not running
        * \throw PhoXiScannerNotFound when PhoXi 3D Scanner with HWIdentification is not available on network
        * \throw UnableToStartAcquisition when connection failed
        */
        void connectCamera(std::string HWIdentification,
                           pho::api::PhoXiTriggerMode mode = pho::api::PhoXiTriggerMode::Software,
                           bool startAcquisition = true);

        /**
        * Disconnect from camera if connected to any.
        */
        void disconnectCamera();

        /**
        * Get frame based on id. If id is negative new image is triggered and new PFrame returned.
        *
        * \note only last triggered frame can be returned - recommended usage is with negative number
        * \param id - frame id to return
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        pho::api::PFrame getPFrame(int id = -1);

        /**
        * Get point cloud
        *
        * \param organized - if true return organized point cloud
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getPointCloud(bool organized = true);

        /**
        * Convert PFrame to point cloud
        *
        * \param organized - if true return organized point cloud
        */
        static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
        getPointCloudFromFrame(pho::api::PFrame frame, bool organized = true);

        /**
        * Convert PFrame to organized point cloud
        *
        * \return organized point cloud
        */
        static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getOrganizedCloudFromFrame(pho::api::PFrame frame);

        /**
        * Convert PFrame to unorganized point cloud
        *
        * \return unorganized point cloud
        */
        static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getUnorganizedCloudFromFrame(pho::api::PFrame frame);

        /**
        * Test if connection to PhoXi 3D Scanner is working
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        void isOk();

        /**
        * Test if connection to PhoXi 3D Scanner is working
        */
        bool isConnected();

        /**
        * Test if PhoXi 3D Scanner is Acquiring
        */
        bool isAcquiring();

        /**
        * Start acquisition
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        * \throw UnableToStartAcquisition if acquisition was not started
        */
        void startAcquisition();

        /**
        * Stop acquisition
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        * \throw UnableToStartAcquisition if acquisition was not stopped
        */
        void stopAcquisition();

        /**
        * Trigger new Image
        *
        * \return @return positive id on success, negative number on failure (-1 Trigger not accepted, -2 Device is not running, -3 Communication Error, -4 WaitForGrabbingEnd is not supported)
        * \note id can be passed to getPFrame method
        */
        int triggerImage(bool waitForGrab = false);

        /**
        * Get hardware identification number of currently connected PhoXi 3D Scanner
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        std::string getHardwareIdentification();

        /**
        * Get supported capturing modes
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        std::vector<pho::api::PhoXiCapturingMode> getSupportedCapturingModes();

        /**
        * Set high resolution (2064 x 1544)
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        void setHighResolution();

        /**
        * Set low resolution (1032 x 772)
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        void setLowResolution();

        /**
        * Set trigger mode
        *
        * \param mode new trigger mode
        * \param startAcquisition if true Acquisition will be started
        * \note if mode is Freerun new PFrames will be triggered immediately after acquisition is started
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        * \throw InvalidTriggerMode when invalid trigger mode is passed
        */
        void setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition = false);

        /**
        * Get trigger mode
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        pho::api::PhoXiTriggerMode getTriggerMode();

    protected:
        pho::api::PPhoXi scanner;
        pho::api::PhoXiFactory phoXiFactory;

        /**
        * Return all PhoXi 3D Scanners ids connected on network.
        *
        * \note returned id can be passed to connectCamera method
        * \throw PhoXiControlNotRunning when PhoXi Control is not running
        */
        std::vector<pho::api::PhoXiDeviceInformation> deviceInforamtionList();

    private:
        int last_frame_id = -1;
    };
}


#endif //PROJECT_PHOXIINTERFACE_H
