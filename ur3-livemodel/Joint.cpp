//
// Created by rob-ot on 13.3.19.
//

#include <Eigen/Dense>
#include "ur3-livemodel/headers/Joint.h"

const Artifact &Joint::getParentArtifact() const {
    return parentArtifact;
}

void Joint::setParentArtifact(const Artifact &parentArtifact) {
    Joint::parentArtifact = parentArtifact;
}

const Artifact &Joint::getChildArtifact() const {
    return childArtifact;
}

void Joint::setChildArtifact(const Artifact &childArtifact) {
    Joint::childArtifact = childArtifact;
}

const Eigen::Vector3d &Joint::getWorldTranslation() const {
    return worldTranslation;
}

const Eigen::Vector3d &Joint::getWorldRotationRpy() const {
    return worldRotationRpy;
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

Joint::Joint(const Artifact &parentArtifact, const Artifact &childArtifact, float jointAngleMax, float jointAngleMin,
             float jointAngle) : parentArtifact(parentArtifact), childArtifact(childArtifact){

    this->parentArtifact = parentArtifact;
    this->childArtifact = childArtifact;
    this->jointAngleMax = jointAngleMax;
    this->jointAngleMin = jointAngleMin;

}

void Joint::computeWorldTransformation() {
    //Use the data to populate the affine transformation Matrix
    Eigen::Affine3d currentTransformation = Eigen::Affine3d::Identity();

    Eigen::AngleAxisd rollAngle(worldRotationRpy[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(worldRotationRpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(worldRotationRpy[2], Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    currentTransformation.prerotate(rotationMatrix);
    currentTransformation.translate(getWorldTranslation());

    setWorldTransformation(currentTransformation.matrix());
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

