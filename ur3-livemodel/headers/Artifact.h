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
#include "CollisionArtifact.h"

/***
 *@Class Artifact, Inherits from Robot Part, represents the solid stl parts of the robot, used to create and render
 * the virtual robot model and the collision model.
 */
class Artifact : public RobotPart{
public:
    Artifact(const std::string &ojectStlFile, std::vector<RobotPart*>* parentListRef, float partRadius, float partLength, int indexInParent);
    virtual ~Artifact();

    const std::string &getOjectStlFile() const;     ///returns a reference to the stl file name

    void setOjectStlFile(const std::string &ojectStlFile);

    float getPartRadius() const;

    void setPartRadius(float partRadius);       ///sets the radius of the cylindrical segement

    float getPartLength() const;                ///the length of the segement

    void setPartLength(float partLength);

    vtkSmartPointer<vtkTransform> getVTKtransform();    //vtk transform used to align the parts in the QVtk viewer for visualization.

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getArtifactPc();

    void transformCollisionArtifacts(Eigen::Affine3d allTransform);

    void addCollisionArtifact(CollisionArtifact *collisionArtifact);

private:
    std::string ojectStlFile;   //Location of the stl file, populated in RobotModelImpl
    vtkSmartPointer<vtkPolyData> objectMesh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr artifactPc;
    std::vector<CollisionArtifact*> collisionArtifacts;
public:
    const std::vector<CollisionArtifact *> &getCollisionArtifacts() const;

public:
    const vtkSmartPointer<vtkPolyData> &getPolyMesh() const;

    void setPolyMesh(const vtkSmartPointer<vtkPolyData> &polyMesh);

private:
    float partRadius;
    float partLength;
};


#endif //REALCOLLABORATIONCAL_ARTIFACT_H
