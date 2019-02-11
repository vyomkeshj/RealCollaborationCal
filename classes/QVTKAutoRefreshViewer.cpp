//
// Created by Vyomkesh Jha on 2019-02-11.
//

#include "QVTKAutoRefreshViewer.h"


void QVTKAutoRefreshViewer::refreshPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPointCloud,
                                              std::string tag) {
    if (!viewer->updatePointCloud(currentPointCloud, tag)) {
        viewer->addPointCloud(currentPointCloud, tag);
    }

}

void QVTKAutoRefreshViewer::initializeWidgetInternal() {
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    SetRenderWindow (viewer->getRenderWindow());
    viewer->setupInteractor (GetInteractor(), GetRenderWindow());
    viewer->addText ("PC VIEWER", 200, 300, "text", 0);
}