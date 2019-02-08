//
// Created by Vyomkesh Jha on 2019-02-07.
//

#ifndef REALCOLLABORATIONCAL_CALIBRATIONUIMODEL_H
#define REALCOLLABORATIONCAL_CALIBRATIONUIMODEL_H

#include <list>
#include <librealsense2/hpp/rs_device.hpp>
#include "RealsenseDeviceProvider.h"

class CloudViewerUiModel {
public:
CloudViewerUiModel() {
initializeCameraList();
}

void initializeCameraList() {
    for (auto&& dev : deviceProvider.getContext().query_devices()) // Query the list of connected RealSense devices
    {
        connectedCameraList.push_back(dev);
        deviceProvider.enableDevice(dev);
    }
}
private:

std::list<rs2::device> connectedCameraList;
std::list<rs2::device> enabledCameraList;
rs2::device calibrationCamera;
RealsenseDeviceProvider deviceProvider;

};

#endif //REALCOLLABORATIONCAL_CALIBRATIONUIMODEL_H
