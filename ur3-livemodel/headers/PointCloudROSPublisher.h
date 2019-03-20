//
// Created by rob-ot on 27.2.19.
//

#ifndef REALCOLLABORATIONCAL_POINTCLOUDROSPUBLISHER_H
#define REALCOLLABORATIONCAL_POINTCLOUDROSPUBLISHER_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/point_cloud.h"
#include <ros/ros.h>

class PointCloudROSPublisher {


public:
    PointCloudROSPublisher();
    void publishPointCloudToRos(const sensor_msgs::PointCloud2 &cloud_msg);
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud);

private:
    ros::Publisher pointCloudPublisher;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    ros::Subscriber sub;

};


#endif //REALCOLLABORATIONCAL_POINTCLOUDROSPUBLISHER_H
