//
// Created by rob-ot on 13.3.19.
//

#ifndef REALCOLLABORATIONCAL_ARTIFACT_H
#define REALCOLLABORATIONCAL_ARTIFACT_H

#include <string>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPerspectiveTransform.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include "RobotPart.h"

//Represents a 3D artifact in the scene
class Artifact : public RobotPart{
public:
    Artifact(const std::string &ojectStlFile, std::vector<RobotPart*>* parentListRef, float partRadius, float partLength, int indexInParent);
    virtual ~Artifact();

    const std::string &getOjectStlFile() const;

    void setOjectStlFile(const std::string &ojectStlFile);

    float getPartRadius() const;


    void setPartRadius(float partRadius);

    float getPartLength() const;

    void setPartLength(float partLength);

    vtkSmartPointer<vtkTransform> getVTKtransform();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getArtifactPc();

private:
    std::string ojectStlFile;
    vtkSmartPointer<vtkPolyData> objectMesh;
    pcl::PolygonMesh polygons;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr artifactPc;

public:
    const vtkSmartPointer<vtkPolyData> &getPolyMesh() const;

    void setPolyMesh(const vtkSmartPointer<vtkPolyData> &polyMesh);

private:
    float partRadius;
    float partLength;
};


#endif //REALCOLLABORATIONCAL_ARTIFACT_H
