//
// Created by Vyomkesh Jha on 2019-04-05.
//

#ifndef REALCOLLABORATIONCAL_COLLISIONARTIFACT_H
#define REALCOLLABORATIONCAL_COLLISIONARTIFACT_H


#include <Eigen/Dense>

/*
 * Represents the line model used to check for collisions of the robot,
 * uses the part's axis as the axis and joint's origin as it's origin.
 *
 * collision is detected by checking if there are distance inlier points to the line below a specified distance.
 * **/
class CollisionArtifact {
public:
    CollisionArtifact(Eigen::Vector3d lineOrigin, Eigen::Vector3d lineAxis, float artifactLength, float artifactRadius);
    void transformArtifact(Eigen::Affine3d transformation);
    bool isInlier(double x, double y, double z);

     Eigen::Vector3d getLineEnd();
     Eigen::Vector3d getLineBegin();

private:
    Eigen::Vector3d lineOrigin;
    Eigen::Vector3d lineAxis;
    float artifactLength;
    float artifactRadius;

};


#endif //REALCOLLABORATIONCAL_COLLISIONARTIFACT_H
