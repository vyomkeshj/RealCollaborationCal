//
// Created by rob-ot on 14.3.19.
//

#ifndef REALCOLLABORATIONCAL_ROBOTMODEL_H
#define REALCOLLABORATIONCAL_ROBOTMODEL_H


#include <string>
#include <vector>
#include "RobotPart.h"

class RobotModel {
public:
    RobotModel();
    void addPart(RobotPart* newPart);
    void rotateAtJoint(int jointIndex, float angle);
    std::vector<RobotPart*>* getPartsInSequence();
    void setPartsInSequence(const std::vector<RobotPart*> &partsInSequence);
private:
std::vector<RobotPart*> partsInSequence;
};


#endif //REALCOLLABORATIONCAL_ROBOTMODEL_H
