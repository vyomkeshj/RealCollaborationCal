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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QtWidgets/QListWidget>
#include <ur3-livemodel/headers/RobotModelImpl.h>

#include "ur3-livemodel/headers/PointCloudROSPublisher.h"

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
    void enableTogglePressed();
    void startCalibration();
    void saveCalibration();

    void rotationZSliderChanged(int sliderval);
    void rotationXSliderChanged(int sliderval);
    void rotationYSliderChanged(int sliderval);

    void translationXSliderChanged(int sliderval);
    void translationYSliderChanged(int sliderval);
    void translationZSliderChanged(int sliderval);

    void updateSelectedDevice(QListWidgetItem *item);
    void repaintPointCloud();
    void updateFrameRobotModel();
private:
    struct afterTransformer{
        double rx;
        double ry;
        double rz;

        double x;
        double y;
        double z;

        Eigen::Matrix4d getNetAffineTransformer() {
            Eigen::Affine3d transformer = Eigen::Affine3d::Identity();
            Eigen::AngleAxisd rollAngle(rz, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd yawAngle(ry, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd pitchAngle(rx, Eigen::Vector3d::UnitX());

            Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
            Eigen::Matrix3d rotationMatrix = q.matrix();
            transformer.prerotate(rotationMatrix);
            transformer.pretranslate(Eigen::Vector3d(x/100,y/100, z/100));

            return transformer.matrix();
        }

        void reset() {
            rx, ry, rz =0;
            x,y,z =0;
        }
    } transformer;
    Ui::VisionsOfJohanna *ui;
    PointCloudROSPublisher pcPublisher;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudFromCamera(int camera);
    bool isCalibrationEnabled = false;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    Eigen::Matrix4d currentTransformer;
    QListWidgetItem *selectedDevice;
    RobotModelImpl implementedRobotModel;
    void keepPointCloudsUpToDate();
    void updateDeviceList();
    void setupSliders();
    void addOrUpdatepointcloud(std::string deviceSerial, Eigen::Matrix4d transform);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr getSegementedPc(pcl::PointCloud <pcl::PointXYZRGB>::Ptr pcIn);
};

#endif
