//
// Created by Vyomkesh Jha on 2019-02-11.
//

#ifndef REALCOLLABORATIONCAL_QVTKAUTOREFRESHVIEWER_H
#define REALCOLLABORATIONCAL_QVTKAUTOREFRESHVIEWER_H


#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKOpenGLNativeWidget.h>

class QVTKAutoRefreshViewer : QVTKOpenGLNativeWidget {
public:
    QVTKAutoRefreshViewer():QVTKOpenGLNativeWidget() {
        initializeWidgetInternal();
    }
    public slots:
    void refreshPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPointCloud, std::string tag);
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    void initializeWidgetInternal();
};


#endif //REALCOLLABORATIONCAL_QVTKAUTOREFRESHVIEWER_H
