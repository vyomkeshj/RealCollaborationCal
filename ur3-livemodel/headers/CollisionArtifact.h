//
// Created by Vyomkesh Jha on 2019-04-05.
//

#ifndef REALCOLLABORATIONCAL_COLLISIONARTIFACT_H
#define REALCOLLABORATIONCAL_COLLISIONARTIFACT_H


#include <Eigen/Dense>

class CollisionArtifact {
public:
    CollisionArtifact(Eigen::Vector3d lineOrigin, Eigen::Vector3d lineAxis, float artifactLength, float artifactRadius);
    void transformArtifact(Eigen::Affine3d transformation);
    bool isInlier(double x, double y, double z);
private:
    Eigen::Vector3d lineOrigin;
    Eigen::Vector3d lineAxis;
    float artifactLength;
    float artifactRadius;

};


#endif //REALCOLLABORATIONCAL_COLLISIONARTIFACT_H
