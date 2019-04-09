//
// Created by rob-ot on 29.1.19.
//

#ifndef REALCOLLABORATIONCAL_CAMERATAGINBASECOORDINATES_H
#define REALCOLLABORATIONCAL_CAMERATAGINBASECOORDINATES_H
//
// Created by rob-ot on 29.1.19.
//
#include <pcl/common/transforms.h>

#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <headers/EigenFileExtension.h>
#include <QtCore/QTextStream>

using namespace pcl;
namespace CameraFrameTransformer {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPcloudWithAffine(  pcl::PointCloud<pcl::PointXYZRGB>::Ptr initial,
                                                                    char* cameraSerial) {
        Eigen::Matrix4d transformer;
        EigenFile::read_binary(cameraSerial, transformer);

        //std::cout<<"transformer= "<<transformer<<"device="<<cameraSerial<<std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::transformPointCloud (*initial, *transformed_cloud, transformer);
        return transformed_cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPcloudWithAffine(  pcl::PointCloud<pcl::PointXYZRGB>::Ptr initial,
                                                                       Eigen::Matrix4d transformer) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
            pcl::transformPointCloud (*initial, *transformed_cloud, transformer);
            return transformed_cloud;

    }

    Eigen::Matrix4d getAffineMatrixForCamera(std::string cameraSerial) {
            Eigen::Matrix4d transformer;
            EigenFile::read_binary( ("/home/rob-ot/Documents/calibration/Camera70540/"+cameraSerial+".dat")
            .c_str(), transformer);
            return transformer;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPcloudWithIcp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr target) {

            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setInputSource(source);
            icp.setInputTarget(target);

            pcl::PointCloud<pcl::PointXYZRGB> sourceToTarget;
            icp.align(sourceToTarget);
            icp.setTransformationEpsilon (1e-8);
            std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                      icp.getFitnessScore() << std::endl;

            std::cout << icp.getFinalTransformation() << std::endl;
            EigenFile::write_binary("icp", icp.getFinalTransformation());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
            pcl::transformPointCloud (*source, *transformed_cloud, icp.getFinalTransformation());
            return transformed_cloud;
    }

};

#endif //REALCOLLABORATIONCAL_CAMERATAGINBASECOORDINATES_H
