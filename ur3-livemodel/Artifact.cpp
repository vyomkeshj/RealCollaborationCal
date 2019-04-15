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
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

#include "ur3-livemodel/headers/Artifact.h"


Artifact::Artifact(const std::string &ojectStlFile, std::vector<RobotPart*>* parentListRef, float partRadius, float partLength, int indexInParent)
: RobotPart(parentListRef, indexInParent) {
    this->ojectStlFile = ojectStlFile;
    vtkSmartPointer<vtkSTLReader> reader =
            vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(ojectStlFile.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();

    actor->SetMapper(mapper);

    objectMesh = vtkPolyData::SafeDownCast(
            actor->GetMapper()->GetInputAsDataSet());

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


Artifact::~Artifact() {

}

const vtkSmartPointer<vtkPolyData> &Artifact::getPolyMesh() const {
    return objectMesh;
}

vtkSmartPointer<vtkTransform> Artifact::getVTKtransform() {
    vtkSmartPointer<vtkMatrix4x4> m =
            vtkSmartPointer<vtkMatrix4x4>::New();
    m->SetElement(0,0,getWorldTransformation().matrix()(0,0));
    m->SetElement(0,1,getWorldTransformation().matrix()(0,1));
    m->SetElement(0,2,getWorldTransformation().matrix()(0,2));
    m->SetElement(0,3,getWorldTransformation().matrix()(0,3));
    m->SetElement(1,0,getWorldTransformation().matrix()(1,0));
    m->SetElement(1,1,getWorldTransformation().matrix()(1,1));
    m->SetElement(1,2,getWorldTransformation().matrix()(1,2));
    m->SetElement(1,3,getWorldTransformation().matrix()(1,3));
    m->SetElement(2,0,getWorldTransformation().matrix()(2,0));
    m->SetElement(2,1,getWorldTransformation().matrix()(2,1));
    m->SetElement(2,2,getWorldTransformation().matrix()(2,2));
    m->SetElement(2,3,getWorldTransformation().matrix()(2,3));
    m->SetElement(3,0,getWorldTransformation().matrix()(3,0));
    m->SetElement(3,1,getWorldTransformation().matrix()(3,1));
    m->SetElement(3,2,getWorldTransformation().matrix()(3,2));
    m->SetElement(3,3,getWorldTransformation().matrix()(3,3));

    vtkSmartPointer<vtkPerspectiveTransform> perspectiveTransform =
            vtkSmartPointer<vtkPerspectiveTransform>::New();
    perspectiveTransform->SetMatrix(m);

    vtkSmartPointer<vtkTransform> transform =
            vtkSmartPointer<vtkTransform>::New();
    transform->SetMatrix(m);
    return transform;
}

void Artifact::setPolyMesh(const vtkSmartPointer<vtkPolyData> &polyMesh) {
    Artifact::objectMesh = polyMesh;
}

void Artifact::addCollisionArtifact(CollisionArtifact *collisionArtifact) {
    collisionArtifacts.emplace_back(collisionArtifact);
}

void Artifact::transformCollisionArtifacts(Eigen::Affine3d allTransform) {
    for(CollisionArtifact* currentCollsionModel : collisionArtifacts) {
        currentCollsionModel->transformArtifact(allTransform);
    }
}

const std::vector<CollisionArtifact *> &Artifact::getCollisionArtifacts() const {
    return collisionArtifacts;
}
