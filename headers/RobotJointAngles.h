//
// Created by rob-ot on 28.1.19.
//

#ifndef REALCOLLABORATION_CAMERATCPORIENTATION_H
#define REALCOLLABORATION_CAMERATCPORIENTATION_H

#define MODBUS_PORT = 502
#include <string.h>
#include <modbus.h>

class RobotJointAngles {
public:
    struct Joints {
        double base ;   ///base joint angle of the robot ...
        double shoulder;
        double elbow;

        double wrist1;
        double wrist2;
        double wrist3;
    };

     RobotJointAngles(string cobotIpAddr);
    ~RobotJointAngles();

    bool initializeModbus();
    Joints getJointAngles();


private:
    string cobotIpAddr; //The ip address of the robot
    modbus bus;
};


#endif //REALCOLLABORATION_CAMERATCPORIENTATION_H
