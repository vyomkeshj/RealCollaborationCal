//
// Created by rob-ot on 14.3.19.
//

#ifndef REALCOLLABORATIONCAL_ROBOTMODEL_H
#define REALCOLLABORATIONCAL_ROBOTMODEL_H

#define PI 3.14
#include <string>
#include <vector>
#include "RobotPart.h"

class RobotModel {
public:

    double prevAngle1 = -PI;
    double prevAngle2 = -PI/2;
    double prevAngle3 = 0.0;
    double prevAngle4 = -PI/2;
    double prevAngle5 = 0;


    RobotModel();

    void addPart(RobotPart *newPart);

    void rotateAtJoint(int jointIndex, float angle);

    std::vector<RobotPart *> *getPartsInSequence();

    void setPartsInSequence(const std::vector<RobotPart *> &partsInSequence);

    void setJointAngles(double angle1, double angle2, double angle3, double angle4, double angle5);

private:
    std::vector<RobotPart *> partsInSequence;

    double radianToLeastRotation(double angle) {
        angle = fmod(angle,2*PI);
        if (angle < 0)
            angle += 360;
        return angle;
    }
};


#endif //REALCOLLABORATIONCAL_ROBOTMODEL_H
