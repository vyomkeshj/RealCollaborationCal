//
// Created by rob-ot on 17.09.19.
//

#ifndef REALCOLLABORATIONCAL_PHOXIINTERFACE_CPP
#define REALCOLLABORATIONCAL_PHOXIINTERFACE_CPP


//
// Created by controller on 1/11/18.
//

#include <headers/PhoXiInterface.h>
#include <headers/PhoXiConversions.h>

namespace phoxi_camera {

    PhoXiInterface::PhoXiInterface() {

    }

    std::vector<PhoXiDeviceInformation> PhoXiInterface::deviceList() {
        if (!phoXiFactory.isPhoXiControlRunning()) {
            scanner.Reset();
            throw PhoXiControlNotRunning("PhoXi Control is not running");
        }
        std::vector<PhoXiDeviceInformation> deviceInfo;
        auto dl = phoXiFactory.GetDeviceList();
        toPhoXiCameraDeviceInforamtion(dl, deviceInfo);
        return deviceInfo;
    }

    std::vector<std::string> PhoXiInterface::cameraList() {

        auto dl = deviceList();
        std::vector<std::string> hwIdentificationList;
        for (const std::string& device : dl) {
            hwIdentificationList.push_back(device);
            std::cout<<"devices connected: "<<device<<std::endl;
        }
        return hwIdentificationList;
//        return std::vector<std::string>();
    }

    void PhoXiInterface::connectCamera(std::string HWIdentification, pho::api::PhoXiTriggerMode mode,
                                       bool startAcquisition) {
        if (this->isConnected()) {
            if (scanner->HardwareIdentification == HWIdentification) {
                this->setTriggerMode(mode, startAcquisition);
                return;
            }
        }
        std::vector<std::string> cl = cameraList();
        auto it = std::find(cl.begin(), cl.end(), HWIdentification);
        if (it == cl.end()) {
            throw PhoXiScannerNotFound("Scanner not found");
        }
        disconnectCamera();
        if (!(scanner = phoXiFactory.CreateAndConnect(*it, 5000))) {
            disconnectCamera();
            throw UnableToStartAcquisition("Scanner was not able to connect. Disconnected.");
        }
        this->setTriggerMode(mode, startAcquisition);
    }

    void PhoXiInterface::disconnectCamera() {
        if (scanner && scanner->isConnected()) {
            scanner->Disconnect(true);
        }
        last_frame_id = -1;
    }

    pho::api::PFrame PhoXiInterface::getPFrame(int id) {
        this->isOk();

        if (id < 0) {
            try {
                id = this->triggerImage(true);
            } catch (UnableToTriggerFrame& e) {
                throw;
            }
        }

        return scanner->GetSpecificFrame(id, 10000);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PhoXiInterface::getPointCloud(bool organized) {
        return getPointCloudFromFrame(getPFrame(-1), organized);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PhoXiInterface::getPointCloudFromFrame(pho::api::PFrame frame, bool organized) {
        if (organized) {
            return getOrganizedCloudFromFrame(frame);
        }
        return getUnorganizedCloudFromFrame(frame);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
    PhoXiInterface::getOrganizedCloudFromFrame(pho::api::PFrame frame) {
        if (!frame || !frame->Successful) {
            throw CorruptedFrame("Corrupted frame!");
        }
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(
                new pcl::PointCloud<pcl::PointXYZ>(frame->GetResolution().Width, frame->GetResolution().Height));
        for (int r = 0; r < frame->GetResolution().Height; r++) {
            for (int c = 0; c < frame->GetResolution().Width; c++) {
                auto point = frame->PointCloud.At(r, c);
                pcl::PointXYZ pclPoint;
                pclPoint.x = point.x / 1000;                 //to [m]
                pclPoint.y = point.y / 1000;                 //to [m]
                pclPoint.z = point.z / 1000;                 //to [m]
                cloud->at(c, r) = pclPoint;
            }
        }
        return cloud;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PhoXiInterface::getUnorganizedCloudFromFrame(pho::api::PFrame frame) {
        if (!frame || !frame->Successful) {
            throw CorruptedFrame("Corrupted frame!");
        }
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (int r = 0; r < frame->GetResolution().Height; r++) {
            for (int c = 0; c < frame->GetResolution().Width; c++) {
                auto point = frame->PointCloud.At(r, c);
                if (point == pho::api::Point3_32f(0, 0, 0)) {
                    continue;
                }
                pcl::PointXYZ pclPoint;
                pclPoint.x = point.x / 1000;                 //to [m]
                pclPoint.y = point.y / 1000;                 //to [m]
                pclPoint.z = point.z / 1000;                 //to [m]
                cloud->push_back(pclPoint);
            }
        }
        return cloud;
    }

    void PhoXiInterface::isOk() {
        if (!scanner || !scanner->isConnected()) {
            throw PhoXiScannerNotConnected("No scanner connected");
        }
    }

    std::string PhoXiInterface::getHardwareIdentification() {
        this->isOk();
        return scanner->HardwareIdentification;
    }

    bool PhoXiInterface::isConnected() {
        return (scanner && scanner->isConnected());
    }

    bool PhoXiInterface::isAcquiring() {
        return (scanner && scanner->isAcquiring());
    }

    void PhoXiInterface::startAcquisition() {
        this->isOk();
        if (scanner->isAcquiring()) {
            return;
        }
        scanner->StartAcquisition();
        if (!scanner->isAcquiring()) {
            throw UnableToStartAcquisition("Unable to start acquisition.");
        }
    }

    void PhoXiInterface::stopAcquisition() {
        this->isOk();
        if (!scanner->isAcquiring()) {
            return;
        }
        scanner->StopAcquisition();
        if (scanner->isAcquiring()) {
            throw UnableToStopAcquisition("Unable to stop acquisition.");
        }
    }

    int PhoXiInterface::triggerImage(bool waitForGrab) {
        this->setTriggerMode(pho::api::PhoXiTriggerMode::Software, true);
        int frame_id = scanner->TriggerFrame(true, waitForGrab);
        last_frame_id = frame_id;

        if (frame_id < 0) {
            switch (frame_id) {
                case -1:
                    throw UnableToTriggerFrame("Trigger not accepted.");
                case -2:
                    throw UnableToTriggerFrame("Device is not running.");
                case -3:
                    throw UnableToTriggerFrame("Communication Error.");
                case -4:
                    throw UnableToTriggerFrame("WaitForGrabbingEnd is not supported.");
                default:
                    throw UnableToTriggerFrame("Unknown error.");
            }
        }
        return frame_id;
    }

    std::vector<pho::api::PhoXiCapturingMode> PhoXiInterface::getSupportedCapturingModes() {
        this->isOk();
        return scanner->SupportedCapturingModes;
    }

    void PhoXiInterface::setHighResolution() {
        this->isOk();
        pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
        mode.Resolution.Width = 2064;
        mode.Resolution.Height = 1544;
        scanner->CapturingMode = mode;
    }

    void PhoXiInterface::setLowResolution() {
        this->isOk();
        pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
        mode.Resolution.Width = 1032;
        mode.Resolution.Height = 772;
        scanner->CapturingMode = mode;
    }

    void PhoXiInterface::setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition) {
        if (!((mode == pho::api::PhoXiTriggerMode::Software) || (mode == pho::api::PhoXiTriggerMode::Hardware) ||
              (mode == pho::api::PhoXiTriggerMode::Freerun) || (mode == pho::api::PhoXiTriggerMode::NoValue))) {
            throw InvalidTriggerMode("Invalid trigger mode " + std::to_string(mode) + ".");
        }
        this->isOk();
        if (mode == scanner->TriggerMode.GetValue()) {
            if (startAcquisition) {
                this->startAcquisition();
            } else {
                this->stopAcquisition();
            }
            return;
        }
        this->stopAcquisition();
        scanner->TriggerMode = mode;
        if (startAcquisition) {
            this->startAcquisition();
        }
    }

    pho::api::PhoXiTriggerMode PhoXiInterface::getTriggerMode() {
        this->isOk();
        return scanner->TriggerMode;
    }

}

#endif //REALCOLLABORATIONCAL_PHOXIINTERFACE_CPP
