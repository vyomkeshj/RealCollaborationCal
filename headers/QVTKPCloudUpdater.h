//
// Created by rob-ot on 31.1.19.
//

#ifndef REALCOLLABORATIONCAL_PCLVISUALISER_H
#define REALCOLLABORATIONCAL_PCLVISUALISER_H

#include "RealsenseManager.h"

#include <QThread>

class QVTKPCloudUpdater :public QThread {
Q_OBJECT
public:
    void run();

    signals:
    //std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::string> emitNewPointcloud();

private:
    RealsenseManager manager;
};


#endif //REALCOLLABORATIONCAL_PCLVISUALISER_H
