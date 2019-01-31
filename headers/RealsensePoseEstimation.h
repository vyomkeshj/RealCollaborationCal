//
// Created by rob-ot on 21.1.19.
//

#ifndef REALCOLLABORATION_REALSENSEPOSEESTIMATION_H
#define REALCOLLABORATION_REALSENSEPOSEESTIMATION_H

#include <opencv2/aruco.hpp>

#include <opencv2/aruco/charuco.hpp>
#include "RealsenseManager.h"
using namespace cv;
class RealsensePoseEstimation {
public:
    RealsensePoseEstimation(String cameraParamsPath, float depthFilter):
    realsenseManager(depthFilter)
    {
        this->cameraParamsPath = cameraParamsPath;
        bool readOk = readCameraParameters(cameraParamsPath, camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
        }
    }

    /**
     * Returns the estimated rotation and translation in the realsense camera's frame,
     * utilizes the depth capability of the realsense camera to improve accuracy of computation
     *
     * Returns @int markerId, @Point3f rotation respect to first marker,
     * @Point3f translation required
     * */
    /*tuple<bool, Point3f, Point3f>*/void getEstimatedPose() {
        while(true) {
            auto[image, depth_information] = this->realsenseManager.getCVAlignedMatrix();

            double tick = (double) getTickCount();

            vector<int> markerIds, charucoIds;
            vector<vector<Point2f> > markerCorners, rejectedMarkers;
            Mat rvec, tvec;

            // detect markers
            aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams,
                                 rejectedMarkers);
            for(auto const& value: markerCorners) {
                for (Point2f currentCorner: value) {
                    cout<<"corner points: x= "<<currentCorner.x<< " y= "<<currentCorner.y<<" z= "<<
                    depth_information.get_distance(currentCorner.x, currentCorner.y)<<endl;
                }
            }
        }
    }

    tuple<Vec3d, Vec3d> visualizeDetectedAxesAndReturnTransformation() {
        Vec3d rvec, tvec;
        while(true) {
            Mat imageCopy;
            auto[image, depth_information] = this->realsenseManager.getCVAlignedMatrix();

            double tick = (double)getTickCount();

            vector< int > markerIds, charucoIds;
            vector< vector< Point2f > > markerCorners, rejectedMarkers;
            vector< Point2f > charucoCorners;

            // detect markers
            aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams,
                                 rejectedMarkers);

            // estimate charuco board pose

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
                calculateArucoPose(0.09, currentMarker, rvec, tvec, depth_information);
            }

            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, 0.1);
            //getEstimatedPose();
            //printMatrix(rvec);
            applyColorMap(imageCopy, imageCopy, COLORMAP_JET);
            imshow("Pose Estimator", imageCopy);
            char key = (char)waitKey(waitTime);
            if(key == 27) break;
        }
        return std::make_tuple(rvec, tvec);
    }

    //TODO:: use the data from realsense to factor for the error in z information
    void calculateArucoPose(float side, const vector<cv::Point2f>& markerCorners, Vec3d& rvec, Vec3d& tvec,
            depth_frame dpInfo) {
        int ptr = 3;
        cout<<dpInfo.get_distance(markerCorners[ptr].x, markerCorners[ptr].y)<<endl;
        vector<cv::Point3f> objecPoints;
        objecPoints.push_back(Point3f(0.0f, 0.0f, 0));
        objecPoints.push_back(Point3f(0.0f, side, 0));
        objecPoints.push_back(Point3f(side, side, 0));
        objecPoints.push_back(Point3f(side, 0.0f, 0));

        // Calculate Rotation and Translation
        bool result = cv::solvePnP(objecPoints, markerCorners, camMatrix, distCoeffs, rvec, tvec);
        cout << "M = "<< endl << " "  << tvec << endl << endl;
    }
private:
    String cameraParamsPath;
    RealsenseManager realsenseManager;
    int dictionaryId = 0;
    bool showRejected = true;
    bool refindStrategy = false;
    int waitTime = 10;
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    Mat camMatrix, distCoeffs;
    double totalTime = 0;
    int totalIterations = 0;

     bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
        FileStorage fs(filename, FileStorage::READ);
        if(!fs.isOpened())
            return false;
        fs["camera_matrix"] >> camMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        return true;
    }

     bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
        FileStorage fs(filename, FileStorage::READ);
        if(!fs.isOpened())
            return false;
        fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
        fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
        fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
        fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
        fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
        fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
        fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
        fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
        fs["minDistanceToBorder"] >> params->minDistanceToBorder;
        fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
        fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
        fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
        fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
        fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
        fs["markerBorderBits"] >> params->markerBorderBits;
        fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
        fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
        fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
        fs["minOtsuStdDev"] >> params->minOtsuStdDev;
        fs["errorCorrectionRate"] >> params->errorCorrectionRate;
        return true;
    }

    void printMatrix(Vec3d buf)
    {
        cout<<buf[0] << "," << buf[1] <<"," <<buf[2]<<endl;
    }
};


#endif //REALCOLLABORATION_REALSENSEPOSEESTIMATION_H
