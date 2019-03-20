//
// Created by rob-ot on 13.3.19.
//

#ifndef REALCOLLABORATIONCAL_ARTIFACT_H
#define REALCOLLABORATIONCAL_ARTIFACT_H

#include <string>
#include <Eigen/Dense>
#include "RobotPart.h"

//Represents a 3D artifact in the scene
class Artifact : public RobotPart{
public:
    Artifact(const std::string &ojectStlFile, float partRadius, float partLength);

    const std::string &getOjectStlFile() const;

    void setOjectStlFile(const std::string &ojectStlFile);

    const Eigen::Vector3d &getWorldTranslation() const;

    void setWorldTranslation(const Eigen::Vector3d &worldTranslation);

    const Eigen::Vector3d &getWorldRotationRpy() const;

    void setWorldRotationRpy(const Eigen::Vector3d &worldRotationRpy);

    const Eigen::Vector3d &getRotationAxis() const;

    float getPartRadius() const;

    void setPartRadius(float partRadius);

    float getPartLength() const;

    void setPartLength(float partLength);

    void computeWorldTransform();
private:
    std::string ojectStlFile;
    float partRadius;
    float partLength;
};


#endif //REALCOLLABORATIONCAL_ARTIFACT_H
