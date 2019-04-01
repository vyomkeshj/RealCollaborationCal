//
// Created by rob-ot on 31.1.19.
//

#include <headers/RealsenseManager.h>

#include <QObject>
#include "QVTKPCloudUpdater.h"


std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::string> QVTKPCloudUpdater::emitNewPointcloud() {
    manager.grabNewFrames();
    std::vector<string> deviceNames = manager.getConnectedDeviceIds();
    for (const string &currentDevice: deviceNames) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc =
                manager.getPointCloudFromCamera(currentDevice);
            return std::make_tuple(currentPc, currentDevice);  //FIXME: emit merged pclouds
        }
    }



void QVTKPCloudUpdater::run() {
    emit emitNewPointcloud();
    sleep(10);
}