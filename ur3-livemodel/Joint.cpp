//
// Created by rob-ot on 13.3.19.
//

#include <Eigen/Dense>

#include "ur3-livemodel/headers/Joint.h"

Joint::Joint(Artifact *parentArtifact, Artifact *childArtifact, std::vector<RobotPart *> *partsList,
              float jointAngleMin , float jointAngleMax, float jointAngle, int indexInParent)
             : RobotPart(partsList, indexInParent){
    this->parentArtifact = parentArtifact;
    this->childArtifact = childArtifact;
    this->jointAngleMax = jointAngleMax;
    this->jointAngleMin = jointAngleMin;
    this->jointAngle = jointAngle;
}

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

/*
 * Gets the transformation for every subsequent part depending on the angle
 * and the axis specified
 */
Eigen::Affine3d Joint::getTransformationForSubsequentParts(double angle) const {
    Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
    if(angle<=jointAngleMax && angle >= jointAngleMin) {
        angle = angle - jointAngle;  //the rotation axis and anchor point change
        transformation = Eigen::Translation3d(worldTranslation) * Eigen::AngleAxisd(angle, rotationAxis) * Eigen::Translation3d(-worldTranslation);
    }
    return transformation;
}

Joint::~Joint() {

}

