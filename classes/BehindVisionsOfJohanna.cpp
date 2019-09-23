//
// Created by rob-ot on 18.09.19.
//
#include "BehindVisionsOfJohanna.h"

#include <headers/CameraTagInBaseCoordinates.h>

void BehindVisionsOfJohanna::stopStreaming() {
    isStreaming = false;
}

void BehindVisionsOfJohanna::startStreaming() {
    isStreaming = true;
    int i = 10;
    while(true) {
        try {
            manager.grabNewFrames(); //updates the frameset in the structure

            std::vector<string> deviceNames = manager.getConnectedDeviceIds();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr net(new pcl::PointCloud<pcl::PointXYZRGB>);

            for (const string &currentDevice: deviceNames) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc =
                        manager.getPointCloudFromCamera(currentDevice);
                *net = *net + *currentPc;
            }
            emit updatePointCloud(net);
        } catch (...) {
            std::cout<<"error"<<std::endl;
        }
    }
        //}
}

void BehindVisionsOfJohanna::setParent(VisionsOfJohanna* johannaHerself) {
    this->johanna = johannaHerself;
}
