//
// Created by rob-ot on 14.3.19.
//

#include <ur3-livemodel/headers/RobotModel.h>
#include <ur3-livemodel/headers/Joint.h>

RobotModel::RobotModel() {

}

void RobotModel::addPart(RobotPart* newPart) {
    partsInSequence.emplace_back(newPart); //Add a new joint or an artifact to the model
}

void RobotModel::rotateAtJoint(int jointIndex, float angle) {
//Rotates all the artifacts and joints starting from jointIndex with the angle specified
    RobotPart *part = partsInSequence.at(jointIndex);
    auto *j = dynamic_cast< Joint *>(part);
    if (j != nullptr) {
        Eigen::Affine3d transform = j->getTransformationForSubsequentParts(angle);

        for (std::size_t i = jointIndex; i < getPartsInSequence()->size(); i++) {
            RobotPart *part = partsInSequence.at(i); //gets reference to the part at the index
            auto artifact = dynamic_cast<Artifact *>(part);
            if(artifact != nullptr) {
                artifact->transformElement(transform); //transforms rest of the joints
                partsInSequence.at(i) = artifact;
            } else {
                auto joint = dynamic_cast<Joint *>(part);
                Eigen::Vector3d newState = transform*joint->getWorldTranslation();
                joint->setWorldTranslation(newState);

                newState = transform.rotation()*joint->getRotationAxis();
                joint->setRotationAxis(newState);
            }
        }
    }
}



void RobotModel::setPartsInSequence(const std::vector<RobotPart *> &partsInSequence) {
    this->partsInSequence = partsInSequence;
}

std::vector<RobotPart*>* RobotModel::getPartsInSequence() {
    return &partsInSequence;
}

void RobotModel::setJointAngles(double angle1, double angle2, double angle3, double angle4, double angle5) {
    rotateAtJoint(1, angle1);
    rotateAtJoint(3, angle2);
    rotateAtJoint(5, angle3);
    rotateAtJoint(7, angle4);
    rotateAtJoint(9, angle5);

    prevAngle1 = angle1;
    prevAngle2 = angle2;
    prevAngle3 = angle3;
    prevAngle4 = angle4;
    prevAngle5 = angle5;
}

bool RobotModel::checkCollisionWithPoint(float x, float y, float z) {
    for(RobotPart* part :partsInSequence) {
        auto artifact = dynamic_cast<Artifact *>(part);
        if(artifact!= nullptr) {
            for(CollisionArtifact* currentCollisionArtifact: artifact->getCollisionArtifacts()) {
                if(currentCollisionArtifact->isInlier(x, y, z))
                return true;
            }
        }
    }
    return false;
}

