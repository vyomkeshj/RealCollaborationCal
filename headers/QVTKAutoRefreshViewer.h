//
// Created by Vyomkesh Jha on 2019-02-11.
//

#ifndef REALCOLLABORATIONCAL_QVTKAUTOREFRESHVIEWER_H
#define REALCOLLABORATIONCAL_QVTKAUTOREFRESHVIEWER_H


#define vtkRenderingCore_AUTOINIT4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeTypeOpenGL,tkRenderingOpenGL)
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)

#include <QVTKWidget.h>
#include <vtkRenderWindow.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class QVTKAutoRefreshViewer : QVTKWidget {
    QOBJECT
public:
    QVTKAutoRefreshViewer(QWidget *parent)
    :QVTKWidget(parent) {
        initializeWidgetInternal();
    }
    public slots:
    void refreshPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPointCloud, std::string tag);
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    void initializeWidgetInternal();
};


#endif //REALCOLLABORATIONCAL_QVTKAUTOREFRESHVIEWER_H
