//
// Created by rob-ot on 18.3.19.
//

#ifndef REALCOLLABORATIONCAL_ROBOTMODELIMPL_H
#define REALCOLLABORATIONCAL_ROBOTMODELIMPL_H
#define PI 3.14

#include <map>
#include "RobotModel.h"

class RobotModelImpl {
public:
    void initializeRobot();
    void setJointAngles(double angle1, double angle2, double angle3, double angle4, double angle5);
    RobotModel &getCurrentRobotState();
    std::vector<RobotPart*>* getPartsInSpace();
    RobotModelImpl();
private:
    RobotModel currentRobotState;
    const float shoulderHeight = 0.1519;
    const float upperArmLength = 0.24365;
    const float forearmLength = 0.21325;
    const float wrist1Length = 0.11235 +0.0925 -0.1198;
    const float wrist2length = 0.08535;
    const float wrist3length = 0.0819;

    std::map<int, RobotPart*> jointIndexKeeper;

};


#endif //REALCOLLABORATIONCAL_ROBOTMODELIMPL_H
