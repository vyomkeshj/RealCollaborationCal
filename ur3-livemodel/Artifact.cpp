//
// Created by rob-ot on 13.3.19.
//

#include <ur3-livemodel/headers/RobotPart.h>
#include <pcl/console/print.h>
#include <pcl/io/vtk_lib_io.h>
#include <ur3-livemodel/headers/Artifact.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>

#include "ur3-livemodel/headers/Artifact.h"


Artifact::Artifact(const std::string &ojectStlFile, float partRadius, float partLength) : RobotPart() {
    this->ojectStlFile = ojectStlFile;
    if (pcl::io::loadPolygonFileSTL (ojectStlFile, objectMesh) == 0)
    {
        PCL_ERROR("Failed to load STL file\n");
    }
    setPartRadius(partRadius);
    setPartLength(partLength);
}


const std::string &Artifact::getOjectStlFile() const {
    return ojectStlFile;
}

float Artifact::getPartRadius() const {
    return partRadius;
}

void Artifact::setPartRadius(float partRadius) {
    Artifact::partRadius = partRadius;
}

float Artifact::getPartLength() const {
    return partLength;
}

void Artifact::setPartLength(float partLength) {
    Artifact::partLength = partLength;
}

pcl::PolygonMesh Artifact::getTransformedObjectMesh(Eigen::Matrix4d transform) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PolygonMesh returnMesh;

    pcl::fromPCLPointCloud2(objectMesh.cloud, cloud);
    //pcl::transformPointCloud(cloud, cloud, transform);
    pcl::toPCLPointCloud2(cloud, returnMesh.cloud);
    return objectMesh;
}


Artifact::~Artifact() {

}
