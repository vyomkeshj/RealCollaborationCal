//
// Created by rob-ot on 18.3.19.
//

#include <ur3-livemodel/headers/RobotModelImpl.h>
#include <ur3-livemodel/headers/Artifact.h>
#include <Eigen/Dense>
#include <ur3-livemodel/headers/Joint.h>
#include "ur3-livemodel/headers/RobotModelImpl.h"

using namespace Eigen;

RobotModelImpl::RobotModelImpl() {
    initializeRobot();
}
void RobotModelImpl::initializeRobot() {

    int jointIndex = 1;
    Artifact* baseArtifact = new Artifact("/home/rob-ot/CLionProjects/RealCollaborationCal/ur3-livemodel/ur3stl/base.stl", 0.075, 0.038);
    baseArtifact->setPartName("baseArtifact");
    baseArtifact->setWorldRotationRpy(Vector3d(0.0, 0.0, 0.0));
    baseArtifact->setWorldTranslation(Vector3d(0.0, 0.0, 0.0));

    currentRobotState.addPart(baseArtifact);

    Artifact* shoulderLink =  new Artifact("/home/rob-ot/CLionProjects/RealCollaborationCal/ur3-livemodel/ur3stl/shoulder.stl", 0.075, 0.178);
    shoulderLink->setPartName("shoulderLink");
    shoulderLink->setWorldRotationRpy(Vector3d(0, 0, 0));
    shoulderLink->setWorldTranslation(Eigen::Vector3d(0, 0, 0));
    Joint* shoulderPanJoint = new Joint(baseArtifact, shoulderLink, -2.0 * PI, +2.0 * PI, 0);
    shoulderPanJoint->setWorldRotationRpy(Eigen::Vector3d(0, PI/2, 0));
    shoulderPanJoint->setRotationAxis(Eigen::Vector3d(0, 1, 0));
    shoulderPanJoint->setWorldTranslation(Eigen::Vector3d(0, 0, 0.1519));

    shoulderLink->computeWorldTransformation();
    shoulderPanJoint->computeWorldTransformation();

    currentRobotState.addPart(shoulderPanJoint);
    currentRobotState.addPart(shoulderLink);
    jointIndexKeeper.insert(std::make_pair(jointIndex, shoulderPanJoint));

    Artifact* upperArmLink = new Artifact("/home/rob-ot/CLionProjects/RealCollaborationCal/ur3-livemodel/ur3stl/upperarm.stl", 0.075, 0.24365);
    upperArmLink->setPartName("upperArmLink");

    upperArmLink->setWorldRotationRpy(Vector3d(0.0, 0.0, 0.0));
    upperArmLink->setWorldTranslation(Vector3d(0.0, 0.0, 0.24365/2));

    Joint* shoulderLiftJoint = new Joint(shoulderLink, upperArmLink, -2.0 * PI, +2.0 * PI, 0);
    shoulderLiftJoint->setWorldRotationRpy(Eigen::Vector3d(0, PI/2, 0));
    shoulderLiftJoint->setRotationAxis(Eigen::Vector3d(0, 1, 0));
    shoulderLiftJoint->setWorldTranslation(Eigen::Vector3d(0, 0.1198, 0));

    upperArmLink->computeWorldTransformation();
    shoulderLiftJoint->computeWorldTransformation();

    currentRobotState.addPart(shoulderLiftJoint);
    currentRobotState.addPart(upperArmLink);
    jointIndex = jointIndex + 2;
    jointIndexKeeper.insert(std::make_pair(jointIndex, shoulderLiftJoint));

    Artifact *forearmLink = new Artifact("/home/rob-ot/CLionProjects/RealCollaborationCal/ur3-livemodel/ur3stl/forearm.stl", 0.075, 0.21325);
    forearmLink->setPartName("forearmLink");
    forearmLink->setWorldRotationRpy(Vector3d(0.0, 0.0, 0.0));
    forearmLink->setWorldTranslation(Vector3d(0.0, 0.0,0.21325/2));


    Joint* elbowJoint = new Joint(upperArmLink, forearmLink, -2.0 * PI, +2.0 * PI, 0);
    elbowJoint->setWorldRotationRpy(Eigen::Vector3d(0.0, 0.0, 0.0));
    elbowJoint->setRotationAxis(Eigen::Vector3d(0, 1, 0));
    elbowJoint->setWorldTranslation(Eigen::Vector3d(0, -0.0925, 0.24365));

    forearmLink->computeWorldTransformation();
    elbowJoint->computeWorldTransformation();

    currentRobotState.addPart(elbowJoint);
    currentRobotState.addPart(forearmLink);
    jointIndex = jointIndex + 2;
    jointIndexKeeper.insert(std::make_pair(jointIndex, elbowJoint));

    Artifact* wrist1Link = new Artifact("/home/rob-ot/CLionProjects/RealCollaborationCal/ur3-livemodel/ur3stl/wrist1.stl", 0.075, 0.12);    //TODO: fix the numbers
    wrist1Link->setPartName("wrist1Link");
    wrist1Link->setWorldRotationRpy(Vector3d(0.0, 0.0, 0.0));
    wrist1Link->setWorldTranslation(Vector3d(0.0, 0.0,0));

    Joint* wrist1Joint = new Joint(upperArmLink, forearmLink, -2.0 * PI, +2.0 * PI, 0);
    wrist1Joint->setWorldRotationRpy(Eigen::Vector3d(0, PI/2, 0));
    wrist1Joint->setRotationAxis(Eigen::Vector3d(0, 1, 0));
    wrist1Joint->setWorldTranslation(Eigen::Vector3d(0, 0, 0.21325));


    wrist1Link->computeWorldTransformation();
    wrist1Joint->computeWorldTransformation();

    currentRobotState.addPart(wrist1Joint);
    currentRobotState.addPart(wrist1Link);
    jointIndex = jointIndex + 2;
    jointIndexKeeper.insert(std::make_pair(jointIndex, wrist1Joint));

    Artifact* wrist2Link = new Artifact("/home/rob-ot/CLionProjects/RealCollaborationCal/ur3-livemodel/ur3stl/wrist2.stl", 0.075, 0.12);
    wrist2Link->setPartName("wrist2Link");
    wrist2Link->setWorldRotationRpy(Vector3d(0.0, 0.0, 0.0));
    wrist2Link->setWorldTranslation(Vector3d(0.0, 0.0,0));

    Joint* wrist2Joint = new Joint(upperArmLink, forearmLink, -2.0 * PI, +2.0 * PI, 0);
    wrist2Joint->setWorldRotationRpy(Eigen::Vector3d(0, 0, 0));
    wrist2Joint->setRotationAxis(Eigen::Vector3d(0, 0, 1));
    wrist2Joint->setWorldTranslation(Eigen::Vector3d(0, 0.11235-0.1198+0.0925, 0));

    wrist2Link->computeWorldTransformation();
    wrist2Joint->computeWorldTransformation();

    currentRobotState.addPart(wrist2Joint);
    currentRobotState.addPart(wrist2Link);
    jointIndex = jointIndex + 2;
    jointIndexKeeper.insert(std::make_pair(jointIndex, wrist2Joint));

    Artifact* wrist3Link = new Artifact("/home/rob-ot/CLionProjects/RealCollaborationCal/ur3-livemodel/ur3stl/wrist3.stl", 0.075, 0.12);
    wrist3Link->setPartName("wrist3Link");
    wrist3Link->setWorldRotationRpy(Vector3d(0.0, 0.0, 0.0));
    wrist3Link->setWorldTranslation(Vector3d(0.0, 0.0,0));

    Joint* wrist3Joint = new Joint(upperArmLink, forearmLink, -2.0 * PI, +2.0 * PI, 0);
    wrist3Joint->setWorldRotationRpy(Eigen::Vector3d(0, 0, 0));
    wrist3Joint->setRotationAxis(Eigen::Vector3d(0, 1, 0));
    wrist3Joint->setWorldTranslation(Eigen::Vector3d(0, 0, 0.08535));

    wrist3Link->computeWorldTransformation();
    wrist3Joint->computeWorldTransformation();

    currentRobotState.addPart(wrist3Joint);
    currentRobotState.addPart(wrist3Link);
    jointIndex = jointIndex + 2;
    jointIndexKeeper.insert(std::make_pair(jointIndex, wrist3Joint));
}


 void RobotModelImpl::rotateAtJoint(int jointIndex, double angle) {
     currentRobotState.rotateAtJoint(jointIndex, angle);
 }

std::vector<RobotPart*> RobotModelImpl::getPartsInSpace() {
    return currentRobotState.getPartsInSequence();
}

