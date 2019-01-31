//
// Created by rob-ot on 28.1.19.
//

#ifndef REALCOLLABORATION_CAMERATCPORIENTATION_H
#define REALCOLLABORATION_CAMERATCPORIENTATION_H

#define MODBUS_PORT = 502
#include <string.h>
#include <modbus.h>

class CameraTCPOrientation {
public:
    struct TCP {
        float x;
        float y;
        float z;

        float rx;
        float ry;
        float rz;
    };
    CameraTCPOrientation(string cobotIpAddr);
    ~CameraTCPOrientation();

    bool initializeModbus();
    TCP getTCPOrientation();


private:
    string cobotIpAddr;
    modbus bus;
};


#endif //REALCOLLABORATION_CAMERATCPORIENTATION_H
