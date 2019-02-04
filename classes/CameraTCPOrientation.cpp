//
// Created by rob-ot on 28.1.19.
//

#include "CameraTCPOrientation.h"


CameraTCPOrientation::CameraTCPOrientation(string cobotIpAddr):
        bus(cobotIpAddr, 502) {
    this->cobotIpAddr = cobotIpAddr;
}

bool CameraTCPOrientation::initializeModbus() {

    if(bus.modbus_connect()) {
        return true;
    }
    return false;
}

CameraTCPOrientation::~CameraTCPOrientation() {
    delete(&bus);
}
CameraTCPOrientation::TCP CameraTCPOrientation::getTCPOrientation() {
    TCP currentTCP;
    uint16_t read_input_regs[1];
    bus.modbus_read_input_registers(400, 1, read_input_regs);
    currentTCP.x = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(401, 1, read_input_regs);
    currentTCP.y = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(402, 1, read_input_regs);
    currentTCP.z = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(403, 1, read_input_regs);
    currentTCP.rx = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(404, 1, read_input_regs);
    currentTCP.ry = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    bus.modbus_read_input_registers(405, 1, read_input_regs);
    currentTCP.rz = (read_input_regs[0]>32768)?(read_input_regs[0]-65535):read_input_regs[0];

    return currentTCP;
}