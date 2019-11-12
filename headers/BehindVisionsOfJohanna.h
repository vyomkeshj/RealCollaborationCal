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
    explicit BehindVisionsOfJohanna(QObject *parent = 0) :QObject(parent), manager(1.5f) {
        qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("pcl::PointCloud<pcl::PointXYZRGB>::Ptr");
}
    void setParent(VisionsOfJohanna *johannaHerself);

public Q_SLOTS:
    void startStreaming();
    void stopStreaming();

    signals:
    inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);


private:
    struct frameData {
        cv::Mat depthFrame;
        std::string timestamp;
        float centralDistance;
    };

    std::vector<frameData> persistance;
    bool persist = false;


    VisionsOfJohanna* johanna;
    RealsenseManager manager;
    bool isStreaming = false;
    double computeFrameCentralDistance(rs2::depth_frame &depthFrame);
    bool isFrameNormal(cv::Mat &depthFrame);
    void persistMatrix(frameData data, int count);
    int burstStore = 40;

};


#endif //REALCOLLABORATIONCAL_BEHINDVISIONSOFJOHANNA_H
