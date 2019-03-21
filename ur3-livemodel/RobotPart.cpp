//
// Created by rob-ot on 20.3.19.
//

#include "headers/RobotPart.h"

void RobotPart::transformElement(Eigen::Matrix4d transform) {
    setWorldTransformation(transform*getWorldTransformation());
}

void RobotPart::setWorldTransformation(const Eigen::Matrix4d &worldTransformation) {
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