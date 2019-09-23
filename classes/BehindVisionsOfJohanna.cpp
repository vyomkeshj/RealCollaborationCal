//
// Created by rob-ot on 18.09.19.
//
#include "BehindVisionsOfJohanna.h"

#include <headers/CameraTagInBaseCoordinates.h>

void BehindVisionsOfJohanna::stopStreaming() {
    isStreaming = false;
}

void BehindVisionsOfJohanna::startStreaming() {
    manager = new RealsenseManager(1.5f);
    isStreaming = true;
    int i = 10;
    for(int j=0; j<i; j++) {
            std::vector<string> deviceNames = manager->getConnectedDeviceIds();
            manager->grabNewFrames(); //updates the frameset in the structure

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr net(new pcl::PointCloud<pcl::PointXYZRGB>);

            for (const string &currentDevice: deviceNames) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc =
                        manager->getPointCloudFromCamera(currentDevice);
                //Fixme: remove redundant code
            }
            emit updatePointCloud(net);
    }
        //}
}

void BehindVisionsOfJohanna::setParent(VisionsOfJohanna* johannaHerself) {
    this->johanna = johannaHerself;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BehindVisionsOfJohanna::updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud) {
    return pointCloud;
}
