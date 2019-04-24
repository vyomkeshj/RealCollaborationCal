# Real Collaboration Cal
## Live obstacle tracking and collision prediction in collaborative robot workspaces.


### Required Libraries:
1. Realsense 2 (Intel Realsense library for the realsense cameras).
2. PCL 1.9.1 (Built with visualisation support)
3. OpenCV 4
4. Qt 5
5. VTK 8.1

## Introduction:
Collaborative robots are supposed to work right alongside their human counterparts, although safety remains
a concern while considering the high speeds (high momentum) the robots may be working at and any collision with the
human worker therefore has the potential to hurt. The current safetly methods employed are usually boundaries imposed
on the workspace and whenever somebody crosses the boundary, the robot responds by either slowing down or stopping, other
methods include force sensors to detect collision and stop the robot.
This project attempts to find a more precise way of predicting such a collision event by live scanning the workspace
of the robot using multiple Intel Realsense D435 cameras (Multiple to get a better scan), combining the data to one scan
in pointcloud form with reference at the robot's origin, segementing the point-cloud to recognise stray objects from the
robot and using this information along with robot's current position and trajectory to predict the collision event in future.



## Steps:

### 1. Calibration:
      * Aruco Based Calibration
      * GUI Based Manual Calibration

#### Aruco Based Calibration:

This method employs aruco markers placed alongside every camera and using another camera (Calibration camera), mounted
at the robot's TCP, the first step involves calibrating the calibration camera to get it's intrinsics, then we use the
intrinsics with open CV to calculate the pose of the aruco tag (also the pose of the corresponding camera) with respect
to the calibration camera. We know the transformation from the calibration camera frame to the robot's origin by using
the TCP pose data provided to us by the robot. We further combine the transformation to get the transformation from the
camera's perspective to robot's base coordinates.

