//
// Created by rob-ot on 27.2.19.
//

#include "ur3-livemodel/headers/PointCloudROSPublisher.h"

void PointCloudROSPublisher::publishPointCloudToRos(const sensor_msgs::PointCloud2 &cloud_msg) {
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(cloud_msg, pcl_pc);
    pcl::fromPCLPointCloud2 (pcl_pc, cloud);
    /*
    Actually, I want to execute some process here.
    But, now, this is for test.
    */
    sensor_msgs::PointCloud2 pc2;
    pcl::PCLPointCloud2::Ptr pcl_pc_2(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2 (cloud, *pcl_pc_2);
    pcl_conversions::fromPCL( *pcl_pc_2, pc2 );
    pointCloudPublisher.publish(pc2);   //this is for publishing sensor_msg::PointCloud2
}

PointCloudROSPublisher::PointCloudROSPublisher() {
    std::map<std::string, std::string> localMessages;
    ros::init (localMessages, "pcl_transfer");

    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe ("scan", 1, &PointCloudROSPublisher::publishPointCloudToRos, this);

    // Create a ROS publisher for the output point cloud
    pointCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/p_cloud", 1);

}


void PointCloudROSPublisher::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud) {
    cloud.header.frame_id = "base";
    pointCloudPublisher.publish(cloud);   //this is for publishing sensor_msg::PointCloud2

    ros::spinOnce();
}
