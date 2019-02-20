//
//  visionsofjohanna.cpp
//  RealCollaborationCal
//
//  Created by Vyomkesh Jha on 07/02/19.
//
#include <headers/RealsenseManager.h>

#include "visionsofjohanna.hpp"
#include <QVTKWidget2.h>
#include <headers/CameraTagInBaseCoordinates.h>
#include "ui_visionsofjohanna.h"

pcl::visualization::PCLVisualizer::Ptr viewer;
RealsenseManager manager(1.0f);

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
   viewer->addText ("Bumblebee", 200, 300, "text", 0);
   keepPointCloudsUpToDate();
   updateDeviceList();
   setupSliders();
   connect (ui->enableDisableCameraButton,  SIGNAL (clicked ()), this, SLOT (enableTogglePressed()));
    connect(ui->calibrateSelectedCameraButton, SIGNAL (clicked()), this, SLOT(startCalibration()));
    connect(ui->saveCalibrationButton, SIGNAL (clicked()), this, SLOT(saveCalibration()));
   connect(ui->cameraListWidget, SIGNAL (itemClicked(QListWidgetItem *)), this, SLOT(updateSelectedDevice(QListWidgetItem *)));
   connect(ui->rz_slider, SIGNAL (valueChanged(int)), this, SLOT(rotationZSliderChanged(int)));
   connect(ui->ry_slider, SIGNAL (valueChanged(int)), this, SLOT(rotationYSliderChanged(int)));
   connect(ui->rx_slider, SIGNAL (valueChanged(int)), this, SLOT(rotationXSliderChanged(int)));
   connect(ui->x_slider, SIGNAL (valueChanged(int)), this, SLOT(translationXSliderChanged(int)));
   connect(ui->y_slider, SIGNAL (valueChanged(int)), this, SLOT(translationYSliderChanged(int)));
   connect(ui->z_slider, SIGNAL (valueChanged(int)), this, SLOT(translationZSliderChanged(int)));


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

void VisionsOfJohanna::enableTogglePressed() {
    printf("Random button was pressed\n");
    keepPointCloudsUpToDate();
}

void VisionsOfJohanna::keepPointCloudsUpToDate() {
    manager.grabNewFrames();
    std::vector<string> deviceNames = manager.getConnectedDeviceIds();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr from;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr to;
        for (const string &currentDevice: deviceNames) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc =
                    manager.getPointCloudFromCamera(currentDevice);
            if(currentDevice == "817612070540") {
                currentPc = CameraFrameTransformer::transformPcloudWithAffine
                (currentPc, "/home/rob-ot/Documents/calibration/Camera70540/817612070540.dat");
                from = currentPc;
                //continue;
            } else if (currentDevice == "817612071554") {
                currentPc = CameraFrameTransformer::transformPcloudWithAffine
                        (currentPc, "/home/rob-ot/Documents/calibration/Camera70540/817612071554.dat");
                to = currentPc;
            }

            if (!viewer->updatePointCloud(currentPc, currentDevice)) {
                viewer->addPointCloud(currentPc, currentDevice);
            }
        }
        //from = CameraFrameTransformer::transformPcloudWithIcp(from, to);
        //if (!viewer->updatePointCloud(from, "icp")) {
        //viewer->addPointCloud(from, "icp");
   //}

    //viewer->resetCamera();
        ui->pclRendererVTKWidget->show();
        ui->pclRendererVTKWidget->update();
}

void VisionsOfJohanna::updateDeviceList() {
    std::vector<string> deviceNames = manager.getConnectedDeviceIds();
    for (const string &currentDevice: deviceNames) {
        QString itemName = QString::fromStdString(currentDevice);;
        ui->cameraListWidget->addItem(new QListWidgetItem(itemName));
    }
}

void VisionsOfJohanna::updateSelectedDevice(QListWidgetItem *item) {
    this->selectedDevice = item;
}

void VisionsOfJohanna::setupSliders() {
    ui->rx_slider->setRange(0, 628);
    ui->ry_slider->setRange(0, 628);
    ui->rz_slider->setRange(0, 628);
    ui->x_slider->setRange(-10000, 10000);
    ui->y_slider->setRange(-10000, 10000);
    ui->z_slider->setRange(-10000, 10000);
}

void VisionsOfJohanna::startCalibration() {
    isCalibrationEnabled = true;
    std::string serial = this->selectedDevice->text().toUtf8().constData();

    EigenFile::read_binary(serial.c_str(), currentTransformer);
}

void VisionsOfJohanna::rotationZSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
    transformer.rz = (sliderval) / 100.000;
    //transform the pointcloud with the new value
    Eigen::Matrix4d netTransform = currentTransformer * transformer.getNetAffineTransformer();
    std::string serial = this->selectedDevice->text().toUtf8().constData();

    addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::rotationYSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.ry = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = currentTransformer * transformer.getNetAffineTransformer();
        std::string serial = this->selectedDevice->text().toUtf8().constData();

        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::rotationXSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.rx = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = currentTransformer * transformer.getNetAffineTransformer();
        std::string serial = this->selectedDevice->text().toUtf8().constData();
        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::translationXSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.x = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = currentTransformer * transformer.getNetAffineTransformer();
        std::string serial = this->selectedDevice->text().toUtf8().constData();
        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::translationYSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.y = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = currentTransformer * transformer.getNetAffineTransformer();
        std::string serial = this->selectedDevice->text().toUtf8().constData();
        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::translationZSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.z = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = currentTransformer * transformer.getNetAffineTransformer();
        std::string serial = this->selectedDevice->text().toUtf8().constData();
        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::addOrUpdatepointcloud(string deviceSerial, Eigen::Matrix4d transform) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc =
            manager.getPointCloudFromCamera(deviceSerial);

    currentPc = CameraFrameTransformer::transformPcloudWithAffine
            (currentPc, transform);

    if (!viewer->updatePointCloud(currentPc, deviceSerial)) {
        viewer->addPointCloud(currentPc, deviceSerial);
    }
    ui->pclRendererVTKWidget->show();
    ui->pclRendererVTKWidget->update();

}

void VisionsOfJohanna::saveCalibration() {
    isCalibrationEnabled = false;
    transformer.reset();
    std::string serial = this->selectedDevice->text().toUtf8().constData();
    Eigen::Matrix4d netTransform = currentTransformer * transformer.getNetAffineTransformer();
    std::string matrixFile = "/home/rob-ot/Documents/calibration/Camera70540/"+serial+"M.dat";
    EigenFile::write_binary(matrixFile.c_str(), netTransform);
}