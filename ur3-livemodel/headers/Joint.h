//
// Created by rob-ot on 13.3.19.
//

#ifndef REALCOLLABORATIONCAL_JOINT_H
#define REALCOLLABORATIONCAL_JOINT_H


#include "Artifact.h"

class Joint : public RobotPart{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Joint();

    virtual ~Joint();

    Joint(Artifact* parentArtifact, Artifact* childArtifact, float jointAngleMax, float jointAngleMin,
          float jointAngle);

    Artifact* getParentArtifact();

    void setParentArtifact(Artifact* parentArtifact);

    Artifact* getChildArtifact();

    void setChildArtifact(Artifact* childArtifact);

    const Eigen::Vector3d &getWorldTranslation() const;


    const Eigen::Vector3d &getWorldRotationRpy() const;

    const Eigen::Vector3d &getRotationAxis() const;

    void setRotationAxis(const Eigen::Vector3d &rotationAxis);

    float getJointAngleMax() const;

    void setJointAngleMax(float jointAngleMax);

    float getJointAngleMin() const;

    void setJointAngleMin(float jointAngleMin);

    Eigen::Matrix4d getTransformationForSubsequentParts(double rotationAngle) const;

private:
    Artifact* parentArtifact;
    Artifact* childArtifact;

    Eigen::Vector3d rotationAxis;
    float jointAngleMax;
    float jointAngleMin;
    float jointAngle;
};


#endif //REALCOLLABORATIONCAL_JOINT_H
