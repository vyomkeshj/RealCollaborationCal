//
//  visionsofjohanna.cpp
//  RealCollaborationCal
//
//  Created by Vyomkesh Jha on 07/02/19.
//
#include <headers/RealsenseManager.h>

#include "visionsofjohanna.hpp"
#include <QVTKWidget.h>
#include <headers/CameraTagInBaseCoordinates.h>
#include "ui_visionsofjohanna.h"
#include <pcl/io/ply_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <ur3-livemodel/headers/Artifact.h>
#include <headers/visionsofjohanna.hpp>
#include <pcl/io/pcd_io.h>

pcl::visualization::PCLVisualizer::Ptr viewer;
RealsenseManager manager(1.0f);

VisionsOfJohanna::VisionsOfJohanna(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::VisionsOfJohanna) {
    ui->setupUi(this);

    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->pclRendererVTKWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->pclRendererVTKWidget->GetInteractor(),
                            ui->pclRendererVTKWidget->GetRenderWindow());
    ui->pclRendererVTKWidget->update();
    viewer->addText("Bumblebee", 0, 0, "text", 0);
    viewer->addCoordinateSystem(1.0);
    jointAnglesListener = new RobotJointAngles("192.168.1.101");  //FIXME: add real ip
    //jointAnglesListener->initializeModbus();
    keepPointCloudsUpToDate();
    updateFrameRobotModel();
    updateDeviceList();
    setupSliders();
    //connect(ui->saveCurrent, SIGNAL (clicked()), this, SLOT (enableTogglePressed()));
    connect(ui->calibrateSelectedCameraButton, SIGNAL (clicked()), this, SLOT(startCalibration()));
    connect(ui->saveCalibrationButton, SIGNAL (clicked()), this, SLOT(saveCalibration()));
    connect(ui->cameraListWidget, SIGNAL (itemClicked(QListWidgetItem * )), this,
            SLOT(updateSelectedDevice(QListWidgetItem * )));
    connect(ui->rz_slider, SIGNAL (valueChanged(int)), this, SLOT(rotationZSliderChanged(int)));
    connect(ui->ry_slider, SIGNAL (valueChanged(int)), this, SLOT(rotationYSliderChanged(int)));
    connect(ui->rx_slider, SIGNAL (valueChanged(int)), this, SLOT(rotationXSliderChanged(int)));
    connect(ui->x_slider, SIGNAL (valueChanged(int)), this, SLOT(translationXSliderChanged(int)));
    connect(ui->y_slider, SIGNAL (valueChanged(int)), this, SLOT(translationYSliderChanged(int)));
    connect(ui->z_slider, SIGNAL (valueChanged(int)), this, SLOT(translationZSliderChanged(int)));
    connect(ui->toggleColorButton, SIGNAL (clicked()), this, SLOT(changePointCloudColorBehaviour()));
    connect(ui->toggleModelButton, SIGNAL (clicked()), this, SLOT(changeModelVisibility()));

    QTimer *pointCloudUpdateTimer = new QTimer(this);
    pointCloudUpdateTimer->setInterval(2 * 100);
    connect(pointCloudUpdateTimer, SIGNAL(timeout()), this, SLOT(repaintPointCloud()));
    pointCloudUpdateTimer->start();
}

VisionsOfJohanna::~VisionsOfJohanna() {
    delete ui;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr VisionsOfJohanna::getPointCloudFromCamera(int camera) {
    manager.grabNewFrames();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_n_cloud_c;
    std::vector<string> deviceNames = manager.getConnectedDeviceIds();
    return manager.getPointCloudFromCamera(deviceNames[camera]);
}

void VisionsOfJohanna::enableTogglePressed() {
    isCurrentPointcloudToBeSaved = true;
}

void VisionsOfJohanna::keepPointCloudsUpToDate() {
    manager.grabNewFrames();
    std::vector<string> deviceNames = manager.getConnectedDeviceIds();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr net(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const string &currentDevice: deviceNames) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc =
                manager.getPointCloudFromCamera(currentDevice);
        if (currentDevice == "817612070540") {
            currentPc = CameraFrameTransformer::transformPcloudWithAffine
                    (currentPc, "/home/rob-ot/Documents/calibration/Camera70540/817612070540.dat");
            if(arePointCloudsColorful)
                tintPointcloud(currentPc, 100, 0 , 0);
            *net = *net+*currentPc;
            //continue;
        } else if (currentDevice == "817612071554") {
            currentPc = CameraFrameTransformer::transformPcloudWithAffine
                    (currentPc, "/home/rob-ot/Documents/calibration/Camera70540/817612071554.dat");
            if(arePointCloudsColorful)
                tintPointcloud(currentPc, 0, 100 , 0);
            *net = *net+*currentPc;

        } else if (currentDevice == "817612070983") {
            currentPc = CameraFrameTransformer::transformPcloudWithAffine
                    (currentPc, "/home/rob-ot/Documents/calibration/Camera70540/817612070983.dat");
            if(arePointCloudsColorful)
                tintPointcloud(currentPc, 0, 0 , 100);
            *net = *net+*currentPc;

        }
        if (!viewer->updatePointCloud(currentPc, currentDevice)) {
            viewer->addPointCloud(currentPc, currentDevice);
        }
        if(isCurrentPointcloudToBeSaved) {
            isCurrentPointcloudToBeSaved = false;
            pcl::io::savePCDFileASCII ("/home/rob-ot/Documents/calibration/Camera70540/saved_pointcloud.pcd", *net);
        }

    }
    //completePc = getSegementedPc(completePc);
    updateFrameRobotModel();

    //pcPublisher.setPointCloud(*completePc);

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
    ui->x_slider->setRange(-30000, 30000);
    ui->y_slider->setRange(-30000, 30000);
    ui->z_slider->setRange(-30000, 30000);
}

void VisionsOfJohanna::startCalibration() {
    isCalibrationEnabled = true;
    std::string serial = this->selectedDevice->text().toUtf8().constData();
    viewer->removePointCloud(serial);
    currentTransformer = CameraFrameTransformer::getAffineMatrixForCamera(serial.c_str());
    //Eigen::Affine3d identity = Eigen::Affine3d::Identity();
    //currentTransformer = identity.matrix();
}

void VisionsOfJohanna::rotationZSliderChanged(int sliderval) {

    if (isCalibrationEnabled) {
        transformer.rz = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = transformer.getNetAffineTransformer()*currentTransformer;
        std::string serial = this->selectedDevice->text().toUtf8().constData();

        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::rotationYSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.ry = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = transformer.getNetAffineTransformer()*currentTransformer;
        std::string serial = this->selectedDevice->text().toUtf8().constData();

        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::rotationXSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.rx = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = transformer.getNetAffineTransformer()*currentTransformer;
        std::string serial = this->selectedDevice->text().toUtf8().constData();
        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::translationXSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.x = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = transformer.getNetAffineTransformer()*currentTransformer;
        std::string serial = this->selectedDevice->text().toUtf8().constData();
        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::translationYSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.y = (sliderval) / 100.000;
        //transform the pointcloud with the new valuesetDistanceThreshold
        Eigen::Matrix4d netTransform = transformer.getNetAffineTransformer()*currentTransformer;
        std::string serial = this->selectedDevice->text().toUtf8().constData();
        addOrUpdatepointcloud(serial, netTransform);
    }
}

void VisionsOfJohanna::translationZSliderChanged(int sliderval) {
    if (isCalibrationEnabled) {
        transformer.z = (sliderval) / 100.000;
        //transform the pointcloud with the new value
        Eigen::Matrix4d netTransform = transformer.getNetAffineTransformer()*currentTransformer;
        std::string serial = this->selectedDevice->text().toUtf8().constData();
        addOrUpdatepointcloud(serial, netTransform);
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr VisionsOfJohanna::getSegementedPc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcIn) {
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pcIn);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 0.6);
    pass.filter(*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(pcIn);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(0.08);
    reg.setPointColorThreshold(3);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(10000);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    return colored_cloud;
}

void VisionsOfJohanna::addOrUpdatepointcloud(string deviceSerial, Eigen::Matrix4d transform) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc =
            manager.getPointCloudFromCamera(deviceSerial);

    currentPc = CameraFrameTransformer::transformPcloudWithAffine
            (currentPc, transform);
    if(arePointCloudsColorful)
    tintPointcloud(currentPc, 100, 100 ,200);
    if (!viewer->updatePointCloud(currentPc, deviceSerial)) {
        viewer->addPointCloud(currentPc, deviceSerial);
    }

    ui->pclRendererVTKWidget->show();
    ui->pclRendererVTKWidget->update();

}

void VisionsOfJohanna::repaintPointCloud() {
    if (!isCalibrationEnabled)
        keepPointCloudsUpToDate();
}

void VisionsOfJohanna::saveCalibration() {
    if(isCalibrationEnabled) {
        isCalibrationEnabled = false;
        std::string serial = this->selectedDevice->text().toUtf8().constData();
        Eigen::Matrix4d netTransform = transformer.getNetAffineTransformer() * currentTransformer;
        std::string matrixFile = "/home/rob-ot/Documents/calibration/Camera70540/" + serial + ".dat";
        EigenFile::write_binary(matrixFile.c_str(), netTransform);
        transformer.reset();
        Eigen::Affine3d identity = Eigen::Affine3d::Identity();
        currentTransformer = identity.matrix();

        ui->rz_slider->setValue(0);
        ui->ry_slider->setValue(0);
        ui->rx_slider->setValue(0);

        ui->z_slider->setValue(0);
        ui->y_slider->setValue(0);
        ui->x_slider->setValue(0);
    }

}

void VisionsOfJohanna::updateFrameRobotModel() {
   // RobotJointAngles::Joints jointStat = jointAnglesListener->getJointAngles();
    //implementedRobotModel.setJointAngles(jointStat.base, jointStat.shoulder, jointStat.elbow,
     //       jointStat.wrist1, jointStat.wrist2);

    viewer->removeAllShapes();
    if(isModelVisible) {
        std::vector<RobotPart *> partsList = *implementedRobotModel.getPartsInSpace();
        for (auto currentPart: partsList) {
            if (currentPart != nullptr) {
                Artifact *currentArtifact = dynamic_cast< Artifact *>(currentPart);
                if (currentArtifact != nullptr) {
                    Eigen::Matrix4d transform = currentArtifact->getWorldTransformation().matrix();
                    viewer->addModelFromPolyData(currentArtifact->getPolyMesh(), currentArtifact->getVTKtransform(),
                                                 currentPart->getPartName());
                }
            }
        }
    }

}
 void VisionsOfJohanna::tintPointcloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr pcld, int r, int g, int b) {
    for(pcl::PointXYZRGB &currentPt: *pcld) {
        currentPt.r = r;
        currentPt.g = g;
        currentPt.b = b;
    }
}

void VisionsOfJohanna::changeModelVisibility() {
    isModelVisible = !isModelVisible;
}

void VisionsOfJohanna::changePointCloudColorBehaviour() {
    arePointCloudsColorful = !arePointCloudsColorful;
}
