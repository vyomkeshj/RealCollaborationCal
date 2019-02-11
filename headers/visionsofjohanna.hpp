//
//  visionsofjohanna.hpp
//  RealCollaborationCal
//
//  Created by Vyomkesh Jha on 07/02/19.
//

#ifndef VISIONSOFJOHANNA_H
#define VISIONSOFJOHANNA_H
#define vtkRenderingCore_AUTOINIT4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeTypeOpenGL,tkRenderingOpenGL)
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)

#include <QMainWindow>
#include <vtkRenderWindow.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace Ui {
    class VisionsOfJohanna;
}

class VisionsOfJohanna : public QMainWindow
{
    Q_OBJECT

public:
    explicit VisionsOfJohanna(QWidget *parent = nullptr);
    ~VisionsOfJohanna();

public Q_SLOTS:
    void
    randomButtonPressed ();

private:
    Ui::VisionsOfJohanna *ui;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudFromCamera(int camera);
    void keepPointCloudsUpToDate();

};

#endif
