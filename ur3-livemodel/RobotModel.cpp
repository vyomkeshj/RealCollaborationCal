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
        Eigen::Matrix4d transform = j->getTransformationForSubsequentParts(angle);

        for (std::size_t i = jointIndex; i < getPartsInSequence()->size(); i++) {
            //TODO: test this, updates all the parts downstream, optimze
            RobotPart *part = partsInSequence.at(i); //gets reference to the part at the index
            auto artifact = dynamic_cast<Artifact *>(part);
            if(artifact != nullptr) {
                artifact->transformElement(transform); //transforms rest of the joints
                partsInSequence.at(jointIndex) = artifact;
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
