//
// Created by rob-ot on 28.1.19.
//

#include "RobotJointAngles.h"


RobotJointAngles::RobotJointAngles(string cobotIpAddr):
        bus(cobotIpAddr, 502) {
    this->cobotIpAddr = cobotIpAddr;
}

bool RobotJointAngles::initializeModbus() {

    if(bus.modbus_connect()) {
        return true;
    }
    return false;
}

RobotJointAngles::~RobotJointAngles() {
    delete(&bus);
}
RobotJointAngles::Joints RobotJointAngles::getJointAngles() {
    Joints currentJointData;
    uint16_t read_input_regs[1];
    bus.modbus_read_input_registers(400, 1, read_input_regs);
    currentJointData.base = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(401, 1, read_input_regs);
    currentJointData.shoulder = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(402, 1, read_input_regs);
    currentJointData.elbow = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(403, 1, read_input_regs);
    currentJointData.wrist1 = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(404, 1, read_input_regs);
    currentJointData.wrist2 = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(405, 1, read_input_regs);
    currentJointData.wrist3 = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    return currentJointData;
}