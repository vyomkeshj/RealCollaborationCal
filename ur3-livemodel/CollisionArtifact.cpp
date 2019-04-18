//
// Created by Vyomkesh Jha on 2019-04-05.
//

#include <ur3-livemodel/headers/CollisionArtifact.h>

CollisionArtifact::CollisionArtifact(Eigen::Vector3d lineOrigin, Eigen::Vector3d lineAxis, float artifactLength,
                                     float artifactRadius) {
    this->lineAxis = lineAxis;
    this->lineOrigin = lineOrigin;
    this->artifactLength = artifactLength;
    this->artifactRadius = artifactRadius;

}
/*
 * The line of collision artifact is represented by the point of origin and the direction vector.
 * **/
void CollisionArtifact::transformArtifact(Eigen::Affine3d transform) {
    Eigen::Vector3d transformedElement = transform*lineOrigin;
    this->lineOrigin = transformedElement;

    transformedElement = transform.rotation()*lineAxis;
    this->lineAxis = transformedElement;
}


/*
 * Calculates distance of the point provided from the artifact line,
 * returns true if the points conforms to the collision definition
 * **/
bool CollisionArtifact::isInlier(double x, double y, double z) {
    Eigen::Vector3d lineOriginToPointVector(x-lineOrigin[0], y-lineOrigin[1], z-lineOrigin[2]);
    double distanceFromOrigin = lineAxis.dot(lineOriginToPointVector);
    Eigen::Vector3d vectorToIntersectionOfPerpendicular = distanceFromOrigin*lineAxis;
    Eigen::Vector3d perpendicularVector = lineOriginToPointVector - vectorToIntersectionOfPerpendicular;

    return distanceFromOrigin > 0 && distanceFromOrigin < artifactLength && perpendicularVector.norm() < artifactRadius;

}
/*
 * The collision artifact line's finite endpoint
 * **/
 Eigen::Vector3d CollisionArtifact::getLineEnd()  {
    double newx= artifactLength*lineAxis[0];
    double newy= artifactLength*lineAxis[1];
    double newz= artifactLength*lineAxis[2];

    return lineOrigin+Eigen::Vector3d(newx, newy, newz);
}

 Eigen::Vector3d CollisionArtifact::getLineBegin()  {
    return lineOrigin;
}
