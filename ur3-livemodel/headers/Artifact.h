//
// Created by rob-ot on 13.3.19.
//

#ifndef REALCOLLABORATIONCAL_ARTIFACT_H
#define REALCOLLABORATIONCAL_ARTIFACT_H

#include <string>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include "RobotPart.h"

//Represents a 3D artifact in the scene
class Artifact : public RobotPart{
public:
    Artifact(const std::string &ojectStlFile, float partRadius, float partLength);
    virtual ~Artifact();

    const std::string &getOjectStlFile() const;

    void setOjectStlFile(const std::string &ojectStlFile);

    float getPartRadius() const;

    pcl::PolygonMesh getTransformedObjectMesh(Eigen::Matrix4d transform);

    void setPartRadius(float partRadius);

    float getPartLength() const;

    void setPartLength(float partLength);

private:
    std::string ojectStlFile;
    pcl::PolygonMesh objectMesh;
    float partRadius;
    float partLength;
};


#endif //REALCOLLABORATIONCAL_ARTIFACT_H
