//
// Created by rob-ot on 13.3.19.
//

#ifndef REALCOLLABORATIONCAL_JOINT_H
#define REALCOLLABORATIONCAL_JOINT_H


#include "Artifact.h"
#include <Eigen/Dense>

class Joint : public RobotPart{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW



    Joint(Artifact* parentArtifact, Artifact* childArtifact,
            std::vector<RobotPart*>* listReference,
             float jointAngleMin, float jointAngleMax,
            float jointAngle, int indexInParent);

    Artifact* getParentArtifact();

    void setParentArtifact(Artifact* parentArtifact);

    Artifact* getChildArtifact();

    void setChildArtifact(Artifact* childArtifact);

    const Eigen::Vector3d &getRotationAxis() const;

    void setRotationAxis(const Eigen::Vector3d &rotationAxis);

    float getJointAngleMax() const;

    void setJointAngleMax(float jointAngleMax);

    float getJointAngleMin() const;

    void setJointAngleMin(float jointAngleMin);

    Eigen::Affine3d getTransformationForSubsequentParts(double rotationAngle) const;

    virtual ~Joint();

private:
    Artifact* parentArtifact;
    Artifact* childArtifact;

    Eigen::Vector3d rotationAxis;
    float jointAngleMax;
    float jointAngleMin;
    float jointAngle;
};


#endif //REALCOLLABORATIONCAL_JOINT_H
