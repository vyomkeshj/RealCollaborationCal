//
// Created by rob-ot on 20.3.19.
//

#include <ur3-livemodel/headers/RobotPart.h>

#include "headers/RobotPart.h"

void RobotPart::transformElement(Eigen::Matrix4d transform) {
    setWorldTransformation(Eigen::Affine3d(transform*(getWorldTransformation().matrix())));
}

void RobotPart::setWorldTransformation(Eigen::Affine3d worldTransformation) {
    this->worldTransformation = worldTransformation;
}


void RobotPart::setWorldTranslation(const Eigen::Vector3d &worldTranslation) {
    this->worldTranslation = worldTranslation;
}


void RobotPart::setWorldRotationRpy(const Eigen::Vector3d &worldRotationRpy) {
    this->worldRotationRpy = worldRotationRpy;
}

RobotPart::RobotPart() {

}

RobotPart::~RobotPart() {

}

const std::string &RobotPart::getPartName() const {
    return partName;
}

void RobotPart::setPartName(const std::string &partName) {
    RobotPart::partName = partName;
}

const Eigen::Affine3d &RobotPart::getWorldTransformation() const {
    return worldTransformation;
}

const Eigen::Vector3d &RobotPart::getWorldTranslation() const {
    return worldTranslation;
}

const Eigen::Vector3d &RobotPart::getWorldRotationRpy() const {
    return worldRotationRpy;
}

void RobotPart::computeWorldTransformation() {
    Eigen::Affine3d transformer = Eigen::Affine3d::Identity();
    Eigen::AngleAxisd rollAngle(worldRotationRpy[2], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(worldRotationRpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(worldRotationRpy[0], Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();
    transformer.prerotate(rotationMatrix);
    transformer.translate(getWorldTranslation());
    setWorldTransformation(transformer);
}

