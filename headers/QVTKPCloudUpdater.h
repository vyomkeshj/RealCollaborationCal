//
// Created by rob-ot on 31.1.19.
//

#ifndef REALCOLLABORATIONCAL_PCLVISUALISER_H
#define REALCOLLABORATIONCAL_PCLVISUALISER_H

#include <QThread>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include "RealsenseManager.h"

class QVTKPCloudUpdater :public QThread {
public:
    void run();

    signals:
    std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::string> emitNewPointcloud();

private:
    RealsenseManager manager;
};


#endif //REALCOLLABORATIONCAL_PCLVISUALISER_H
