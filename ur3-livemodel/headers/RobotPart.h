//
// Created by rob-ot on 18.3.19.
//

#ifndef REALCOLLABORATIONCAL_ROBOTPART_H
#define REALCOLLABORATIONCAL_ROBOTPART_H
#include <Eigen/Dense>

 class RobotPart {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RobotPart();
    void transformElement(Eigen::Matrix4d transform);
    const Eigen::Affine3d &getWorldTransformation() const;
    void setWorldTransformation(Eigen::Affine3d worldTransformation);

     const Eigen::Vector3d &getWorldTranslation() const;

     void setWorldTranslation(const Eigen::Vector3d &worldTranslation);

     const Eigen::Vector3d &getWorldRotationRpy() const;

     void setWorldRotationRpy(const Eigen::Vector3d &worldRotationRpy);

     void computeWorldTransformation();

     const std::string &getPartName() const;

     void setPartName(const std::string &partName);

     virtual ~RobotPart();

 protected:
    Eigen::Vector3d worldTranslation;
    Eigen::Vector3d worldRotationRpy;
    Eigen::Affine3d worldTransformation;
    std::string partName;
 };


#endif //REALCOLLABORATIONCAL_ROBOTPART_H
