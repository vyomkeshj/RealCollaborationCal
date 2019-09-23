//
// Created by rob-ot on 18.09.19.
//

#ifndef REALCOLLABORATIONCAL_BEHINDVISIONSOFJOHANNA_H
#define REALCOLLABORATIONCAL_BEHINDVISIONSOFJOHANNA_H


#include "RealsenseManager.h"
#include "visionsofjohanna.hpp"

#include <QtCore/QObject>

/**
 * Manage the data streams to the UI and perform computations on a worker thread
 * Communicates with the main UI thread using signals
 * */
class BehindVisionsOfJohanna : public QObject {
Q_OBJECT

public:
    explicit BehindVisionsOfJohanna(QObject *parent = 0) :QObject(parent) {}
    void setParent(VisionsOfJohanna *johannaHerself);

public Q_SLOTS:
    void startStreaming();
    void stopStreaming();

    signals:
    inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);


private:
    VisionsOfJohanna* johanna;
    RealsenseManager* manager;
    bool isStreaming = false;

};


#endif //REALCOLLABORATIONCAL_BEHINDVISIONSOFJOHANNA_H
