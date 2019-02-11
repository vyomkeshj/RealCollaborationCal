//
//  visionsofjohanna.cpp
//  RealCollaborationCal
//
//  Created by Vyomkesh Jha on 07/02/19.
//
#include <headers/RealsenseManager.h>

#include "visionsofjohanna.hpp"
#include <QVTKWidget2.h>
#include "ui_visionsofjohanna.h"

pcl::visualization::PCLVisualizer::Ptr viewer;
RealsenseManager manager(1.5f);

VisionsOfJohanna::VisionsOfJohanna(QWidget *parent) :
QMainWindow(parent),
ui(new Ui::VisionsOfJohanna)
{
   ui->setupUi(this);
   viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
   ui->pclRendererVTKWidget->SetRenderWindow (viewer->getRenderWindow());
   viewer->setupInteractor (ui->pclRendererVTKWidget->GetInteractor(),
           ui->pclRendererVTKWidget->GetRenderWindow());
   ui->pclRendererVTKWidget->update();
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloudptr = getPointCloudFromCamera(0);
   viewer->addText ("Bumblebee", 200, 300, "text", 0);
    keepPointCloudsUpToDate();
   //viewer->addPointCloud (getPointCloudFromCamera(0), "cloud");
   //viewer->resetCamera();

   connect (ui->enableDisableCameraButton,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

}

VisionsOfJohanna::~VisionsOfJohanna()
{
    delete ui;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr VisionsOfJohanna::getPointCloudFromCamera(int camera) {
    manager.grabNewFrames();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_n_cloud_c;
    std::vector<string> deviceNames = manager.getConnectedDeviceIds();
    return manager.getPointCloudFromCamera(deviceNames[camera]);
}

void VisionsOfJohanna::randomButtonPressed () {
    printf("Random button was pressed\n");
    keepPointCloudsUpToDate();
}


void VisionsOfJohanna::keepPointCloudsUpToDate() {
    manager.grabNewFrames();
    std::vector<string> deviceNames = manager.getConnectedDeviceIds();
    for(const string &currentDevice: deviceNames) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc =
                manager.getPointCloudFromCamera(currentDevice);
        if(!viewer->updatePointCloud (currentPc, currentDevice)) {
            viewer->addPointCloud (currentPc, currentDevice);
        }
    }
    viewer->resetCamera();
    ui->pclRendererVTKWidget->show();
    ui->pclRendererVTKWidget->update();


}