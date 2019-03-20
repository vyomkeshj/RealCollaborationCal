//
// Created by rob-ot on 18.3.19.
//

#ifndef REALCOLLABORATIONCAL_ROBOTPART_H
#define REALCOLLABORATIONCAL_ROBOTPART_H
#include <Eigen/Dense>

 class RobotPart {
public:
    RobotPart();
    void transformElement(Eigen::Matrix4d transform);
    const Eigen::Matrix4d &getWorldTransformation() const;
    void setWorldTransformation(const Eigen::Matrix4d &worldTransformation);

     const Eigen::Vector3d &getWorldTranslation() const;

     void setWorldTranslation(const Eigen::Vector3d &worldTranslation);

     const Eigen::Vector3d &getWorldRotationRpy() const;

     void setWorldRotationRpy(const Eigen::Vector3d &worldRotationRpy);

 protected:
    Eigen::Vector3d worldTranslation;
    Eigen::Vector3d worldRotationRpy;
    Eigen::Matrix4d worldTransformation;

 };


#endif //REALCOLLABORATIONCAL_ROBOTPART_H
