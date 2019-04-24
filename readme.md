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

##### Code Explanation:
[RealsensePoseEstimation.h] (https://github.com/vyomkeshj/RealCollaborationCal/blob/qt_ui/headers/RealsensePoseEstimation.h)

```
/**
* Gets the transformation from aruco to calibration camera given the calibration camera id
* @Param calibcamId
*/
tuple<Vec3d, Vec3d> visualizeDetectedAxesAndReturnTransformation(String calibcamId) {
Vec3d rvec, tvec;
while(true) {
Mat imageCopy;
realsenseManager.getConnectedDeviceIds();
auto[image, depth_information, video_frame] = this->realsenseManager.getCVAlignedMatrix(calibcamId); //Returns aligned video frame and depth frame

double tick = (double)getTickCount();

vector< int > markerIds;
vector< vector< Point2f > > markerCorners, rejectedMarkers; //To store the detected marker corners and rejected markers

// detect markers in the image
aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams,
rejectedMarkers);

double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
totalTime += currentTime;
totalIterations++;
if(totalIterations % 30 == 0) {
cout << "Detection Time = " << currentTime * 1000 << " ms "
<< "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
}

// draw results
image.copyTo(imageCopy);
if(markerIds.size() > 0) {
aruco::drawDetectedMarkers(imageCopy, markerCorners);
}
if(showRejected && rejectedMarkers.size() > 0)
aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));

for (auto& currentMarker: markerCorners) {
calculateArucoPose(0.09, currentMarker, rvec, tvec, depth_information);   //estimate pose for the aruco marker
}

aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, 0.1);
//getEstimatedPose();
//printMatrix(rvec);
//applyColorMap(imageCopy, imageCopy, COLORMAP_JET);
imshow("Pose Estimator", imageCopy);    //visualize result
char key = (char)waitKey(waitTime);
if(key == 27) break;
}
return std::make_tuple(rvec, tvec);
}
```


##### Learnings:
1. [Using a single aruco marker to estimate the pose of the camera is inappropriate] (https://stackoverflow.com/questions/51709522/unstable-values-in-aruco-pose-estimation?noredirect=1&lq=1)
2. Aruco marker detection requires a brightly lit environment.

##### Next:

1. Use multiple markers to improve detection accuracy.

#### GUI Based Manual Calibration:

This method uses a GUI to allow the user to move point-clouds in space and align them in the required fashion,
the user is provided with 6 silders, 3 for rotation and 3 for translation, by changing the slider values, the user can calibrate
the pointclouds to one origin.

##### Steps to calibrate:
1. Choose the camera to be calibrated.
2. Toggle Robot's virtual model/ Toggle tinting of the point-clouds to your preference.
3. Click Start-Calibration
4. Move the translation sliders such that the Robot's origin in the pointcloud coincides with the origin in the visualisation.
5. Move the rotation sliders to rotate the clouds correctly along the axes.
6. Click save-calibration and repeat for other cameras.

Tips To Calibrate:
* Translate before rotate
* Use the mouse-wheel to perform precise movements.
* Check the tinted view for a better visualization of the overlap.
* While rotating/translating about/along an axis, set the visualiser in such a way that the movement is clear, ie
  when rotating about x axis, it's better to look along y or x axis pointing outwards the screen.


##### Code Explanation:
[VisionsOfJohanna.cpp](https: //github.com/vyomkeshj/RealCollaborationCal/blob/qt_ui/classes/VisionsOfJohanna.cpp)
` Eigen:: Matrix4d currentTransformer; //contains the previous saved transformation, read from the file `

```

/**
* Structure storing the data from the sliders, getNetAffineTransformer() returns the transformation matrix corresponding
* to the slider values and the current transformation of the rendered point-cloud is set to getNetAffineTransformer() * currentTransformer.
* upon saveCalibration, this new transformation is saved to file and it becomes the currentTransformer for next time.
*/
struct afterTransformer{
double rx = 0;
double ry = 0;
double rz = 0;

double x = 0;
double y = 0;
double z = 0;

Eigen:: Matrix4d getNetAffineTransformer() {
Eigen:: Affine3d transformer = Eigen::Affine3d:: Identity();
Eigen:: AngleAxisd rollAngle(rz, Eigen:: Vector3d:: UnitZ());
Eigen:: AngleAxisd yawAngle(ry, Eigen:: Vector3d:: UnitY());
Eigen:: AngleAxisd pitchAngle(rx, Eigen:: Vector3d:: UnitX());

Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
Eigen:: Matrix3d rotationMatrix = q.matrix();
transformer.pretranslate(Eigen::Vector3d(x/100, y/100, z/100));
transformer.prerotate(rotationMatrix);

return transformer.matrix();
}
}
```

