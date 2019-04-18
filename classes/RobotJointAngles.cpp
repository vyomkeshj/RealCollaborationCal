//
// Created by rob-ot on 28.1.19.
//

#include "RobotJointAngles.h"


RobotJointAngles::RobotJointAngles(string cobotIpAddr):
        bus(cobotIpAddr, 502) {
    this->cobotIpAddr = cobotIpAddr;
}

/*
 * Must be called after the constructor, connects to the Robot using Modbus
 * **/
bool RobotJointAngles::initializeModbus() {

    if(bus.modbus_connect()) {
        return true;
    }
    return false;
}

RobotJointAngles::~RobotJointAngles() {
    delete(&bus);
}

/*
 * Reads the Modbus registers to check for the joint angles, angles are returned in mrad
 * **/
RobotJointAngles::Joints RobotJointAngles::getJointAngles() {
    Joints currentJointData;
    uint16_t read_input_regs[1];
    bus.modbus_read_holding_registers(270, 1, read_input_regs);
    if(read_input_regs[0]>32768) {
        currentJointData.base = (read_input_regs[0]-65535)/1000.0;
    } else {
        currentJointData.base = read_input_regs[0]/1000.0;
    }
    //currentJointData.base = ((read_input_regs[0]>32768.0) ? (read_input_regs[0]-65535.0):read_input_regs[0])/1000.0;
    //currentJointData.base = ((test>32768.0) ? (test-65535.0):test)/1000.0;

    bus.modbus_read_holding_registers(271, 1, read_input_regs);
    //currentJointData.shoulder = ((read_input_regs[0]>32768.0)?(read_input_regs[0]-65535.0):read_input_regs[0])/1000.0;
    if(read_input_regs[0]>32768) {
        currentJointData.shoulder = (read_input_regs[0]-65535)/1000.0;
    } else {
        currentJointData.shoulder = read_input_regs[0]/1000.0;
    }

    bus.modbus_read_holding_registers(272, 1, read_input_regs);
    //currentJointData.elbow = ((read_input_regs[0]>32768.0)?(read_input_regs[0]-65535.0):read_input_regs[0])/1000.0;
    if(read_input_regs[0]>32768) {
        currentJointData.elbow = (read_input_regs[0]-65535)/1000.0;
    } else {
        currentJointData.elbow = read_input_regs[0]/1000.0;
    }

    bus.modbus_read_holding_registers(273, 1, read_input_regs);
    //currentJointData.wrist1 = ((read_input_regs[0]>32768.0)?(read_input_regs[0]-65535.0):read_input_regs[0])/1000.0;
    if(read_input_regs[0]>32768) {
        currentJointData.wrist1 = (read_input_regs[0]-65535)/1000.0;
    } else {
        currentJointData.wrist1 = read_input_regs[0]/1000.0;
    }

    bus.modbus_read_holding_registers(274, 1, read_input_regs);
    //currentJointData.wrist2 = ((read_input_regs[0]>32768.0)?(read_input_regs[0]-65535.0):read_input_regs[0])/1000.0;
    if(read_input_regs[0]>32768) {
        currentJointData.wrist2 = (read_input_regs[0]-65535)/1000.0;
    } else {
        currentJointData.wrist2 = read_input_regs[0]/1000.0;
    }

    bus.modbus_read_holding_registers(275, 1, read_input_regs);
    //currentJointData.wrist3 = ((read_input_regs[0]>32768.0)?(read_input_regs[0]-65535.0):read_input_regs[0])/1000.0;
    if(read_input_regs[0]>32768) {
        currentJointData.wrist3 = (read_input_regs[0]-65535)/1000.0;
    } else {
        currentJointData.wrist3 = read_input_regs[0]/1000.0;
    }

    return currentJointData;
}