//
// Created by rob-ot on 13.3.19.
//

#ifndef REALCOLLABORATIONCAL_JOINT_H
#define REALCOLLABORATIONCAL_JOINT_H


#include "Artifact.h"

class Joint : public RobotPart{

public:
    Joint();

    Joint(const Artifact &parentArtifact, const Artifact &childArtifact, float jointAngleMax, float jointAngleMin,
          float jointAngle);

    const Artifact &getParentArtifact() const;

    void setParentArtifact(const Artifact &parentArtifact);

    const Artifact &getChildArtifact() const;

    void setChildArtifact(const Artifact &childArtifact);

    const Eigen::Vector3d &getWorldTranslation() const;

    void setWorldTranslation(const Eigen::Vector3d &worldTranslation);

    const Eigen::Vector3d &getWorldRotationRpy() const;

    void setWorldRotationRpy(const Eigen::Vector3d &worldRotationRpy);

    const Eigen::Vector3d &getRotationAxis() const;

    void setRotationAxis(const Eigen::Vector3d &rotationAxis);

    float getJointAngleMax() const;

    void setJointAngleMax(float jointAngleMax);

    float getJointAngleMin() const;

    void setJointAngleMin(float jointAngleMin);

    void computeWorldTransformation();

    Eigen::Matrix4d getTransformationForSubsequentParts(double rotationAngle) const;

private:
    Artifact parentArtifact;
    Artifact childArtifact;

    Eigen::Vector3d rotationAxis;
    float jointAngleMax;
    float jointAngleMin;
    float jointAngle;
};


#endif //REALCOLLABORATIONCAL_JOINT_H
