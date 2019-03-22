//
// Created by rob-ot on 13.3.19.
//

#include <Eigen/Dense>
#include <ur3-livemodel/headers/Joint.h>

#include "ur3-livemodel/headers/Joint.h"

 Artifact* Joint::getParentArtifact()  {
    return parentArtifact;
}

void Joint::setParentArtifact(Artifact* parentArtifact) {
    Joint::parentArtifact = parentArtifact;
}

Artifact* Joint::getChildArtifact()  {
    return childArtifact;
}

void Joint::setChildArtifact(Artifact* childArtifact) {
    Joint::childArtifact = childArtifact;
}

const Eigen::Vector3d &Joint::getRotationAxis() const {
    return rotationAxis;
}

void Joint::setRotationAxis(const Eigen::Vector3d &rotationAxis) {
    Joint::rotationAxis = rotationAxis;
}

float Joint::getJointAngleMax() const {
    return jointAngleMax;
}

void Joint::setJointAngleMax(float jointAngleMax) {
    Joint::jointAngleMax = jointAngleMax;
}

float Joint::getJointAngleMin() const {
    return jointAngleMin;
}

void Joint::setJointAngleMin(float jointAngleMin) {
    Joint::jointAngleMin = jointAngleMin;
}

Joint::Joint(Artifact* parentArtifact, Artifact* childArtifact, float jointAngleMax, float jointAngleMin,
             float jointAngle) {

    this->parentArtifact = parentArtifact;
    this->childArtifact = childArtifact;
    this->jointAngleMax = jointAngleMax;
    this->jointAngleMin = jointAngleMin;

}


/*
 * Gets the transformation for every subsequent part depending on the angle
 * and the axis specified
 */
Eigen::Matrix4d Joint::getTransformationForSubsequentParts(double angle) const {
    Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
    if(angle<=jointAngleMax && angle >= jointAngleMin) {

        angle = angle - jointAngle;  //FIXME: calculate the angle to be rotated

        Eigen::Matrix3d mat;
        mat = Eigen::AngleAxisd(angle, rotationAxis);
        transformation.prerotate(mat);
    } else {

    }
    return transformation.matrix();
}

Joint::~Joint() {

}