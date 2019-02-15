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
#include <headers/EigenFileExtension.h>

namespace CameraFrameTransformer {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPcloudWithAffine(  pcl::PointCloud<pcl::PointXYZRGB>::Ptr initial,
                                                                    char* cameraSerial) {
        Eigen::Matrix4d transformer;
        EigenFile::read_binary(cameraSerial, transformer);
        cout<<"transformer= "<<transformer<<endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::transformPointCloud (*initial, *transformed_cloud, transformer);
        return transformed_cloud;
    }
};


#endif //REALCOLLABORATIONCAL_CAMERATAGINBASECOORDINATES_H
