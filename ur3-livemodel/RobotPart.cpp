//
// Created by rob-ot on 20.3.19.
//

#include <ur3-livemodel/headers/RobotPart.h>
#include <ur3-livemodel/headers/Joint.h>

#include "headers/RobotPart.h"

void RobotPart::transformElement(Eigen::Affine3d transform) {
    Eigen::Affine3d newTransform = Eigen::Affine3d::Identity();
    newTransform.matrix() = transform*(worldTransformation.matrix());
    setWorldTransformation(newTransform);

    auto artifact = dynamic_cast<Artifact *>(this);
    if(artifact!=nullptr)
        artifact->transformCollisionArtifacts(transform); /// In case of artifact, update collision models as well!

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
    Eigen::AngleAxisd rollAngle(worldRotationRpy[2], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(worldRotationRpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(worldRotationRpy[0], Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();
    transformer.prerotate(rotationMatrix);
    transformer.pretranslate(getWorldTranslation());


    if(indexInParentVector-1 >= 0) {
        RobotPart *prevPart = partsList->at(getIndexInParentVector() - 1);
        auto *previousJoint = dynamic_cast<Joint*>(prevPart);

        if (previousJoint != nullptr) {
            Eigen::Matrix4d completeTransformation =
            previousJoint->getWorldTransformation().matrix()*transformer.matrix();
            //setWorldTransformationMatrix(completeTransformation);
            transformer.matrix() = completeTransformation;
        }
    }
    setWorldTransformation(transformer);

}

RobotPart::RobotPart(std::vector<RobotPart *> *partsList, int indexInParent) {
    this->partsList = partsList;
    this->indexInParentVector = indexInParent;
}

int RobotPart::getIndexInParentVector() const {
    return indexInParentVector;
}

void RobotPart::setIndexInParentVector(int indexInParentVector) {
    this->indexInParentVector = indexInParentVector;
}

const Eigen::Matrix4d &RobotPart::getWorldTransformationMatrix() const {
    return worldTransformationMatrix;
}

void RobotPart::setWorldTransformationMatrix(const Eigen::Matrix4d &worldTransformationMatrix) {
    RobotPart::worldTransformationMatrix = worldTransformationMatrix;
}
