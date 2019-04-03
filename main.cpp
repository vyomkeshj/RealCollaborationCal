#include <RobotJointAngles.h>
#include <RealsensePoseEstimation.h>
#include <Eigen/Geometry>
#include "headers/EigenFileExtension.h"
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;
using namespace cv::aruco;
using namespace rs2;

void remove_background(rs2::video_frame& color, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
void totalTransformation(CameraTCPOrientation::TCP currentTCP, Vec3d rvec, Vec3d tvec);
void printMatrix(Vec3d buf);
Eigen::Matrix4d createAffineFromRotationAndTranslationCam(Vec3d rotation, Vec3d translation);
Eigen::Matrix4d createAffineFromRotationAndTranslationRob(Vec3d rotation, Vec3d translation);

int main() {

    RealsensePoseEstimation poseEstimation("/home/rob-ot/Documents/calibration/CameraA/camCalibParams.xml", 1.5f);
    auto[rvec, tvec] = poseEstimation.visualizeDetectedAxesAndReturnTransformation("817612070983");

    CameraTCPOrientation orientation("192.168.1.101");
    if(orientation.initializeModbus()) {
        CameraTCPOrientation::TCP currentTCP = orientation.getTCPOrientation();
        cout<<"x= "<<currentTCP.x<<" y=" <<currentTCP.y<<" z= "<<currentTCP.z<<endl;
        cout<<"rx= "<<currentTCP.rx<<" ry=" <<currentTCP.ry<<" rz= "<<currentTCP.rz<<endl;
        currentTCP.rx = 1747;
        currentTCP.ry= 4349;
        currentTCP.rz= -1288;
        totalTransformation(currentTCP, rvec, tvec);
    }

    return 0;
}

void totalTransformation(CameraTCPOrientation::TCP currentTCP, Vec3d rvec, Vec3d tvec) {

    Eigen::Matrix4d transformationCam = createAffineFromRotationAndTranslationCam(rvec, tvec);
    cout<<"Translation cam= "<<tvec[0]<<","<<tvec[1]<<","<<tvec[2]<<endl;
    cout<<"Rotation Cam = ";
    printMatrix(rvec);
    cout<<endl;

    rvec = cv::Vec3d(currentTCP.rx/1000.000,  currentTCP.ry/1000.000, currentTCP.rz/1000.000);
    tvec = cv::Vec3d(currentTCP.x/10000.00,  currentTCP.y/10000.00, currentTCP.z/10000.00);
    cout<<"Rotation base= "<<currentTCP.rx/1000<<","<<currentTCP.ry/1000<<","<<currentTCP.rz/1000<<endl;

    cout<<"Translation base= "<<currentTCP.x/10000<<","<<currentTCP.y/10000<<","<<currentTCP.z/10000<<endl;
    Eigen::Matrix4d transformationBase = createAffineFromRotationAndTranslationRob(rvec, tvec);

    Eigen::Matrix4d netTransformation = transformationBase*transformationCam;
    cout<<"net transformation before inverse= "<<netTransformation<<endl;

    Eigen::Affine3d affineTransformer(netTransformation);
    affineTransformer = affineTransformer.inverse();
    netTransformation = affineTransformer.matrix();
    cout<<"net transformation inverse= "<<netTransformation<<endl;
    EigenFile::write_binary("/home/rob-ot/Documents/calibration/Camera70540/817612071554.dat", netTransformation);
    }

Eigen::Matrix4d createAffineFromRotationAndTranslationRob(Vec3d rotation, Vec3d translation) {

    double angleMag = sqrt(pow(rotation[0],2)+pow(rotation[1],2)
                           +pow(rotation[2],2));
    Eigen::Vector3d rotationN = Eigen::Vector3d(rotation[0], rotation[1], rotation[2]);
    rotationN.normalize();
    cout<<"magnitude = " <<angleMag<<endl;
    Eigen::Affine3d rx =
            Eigen::Affine3d(Eigen::AngleAxisd(angleMag, rotationN));

    Eigen::Affine3d translation3d(Eigen::Translation3d(Eigen::Vector3d(translation[0]
            ,translation[1],translation[2])));
    Eigen::Matrix4d m = (translation3d * rx).matrix(); // Option 1
    cout<<"Robot affine = "<<m<<endl;
    return m;

}

Eigen::Matrix4d createAffineFromRotationAndTranslationCam(Vec3d rotation, Vec3d translation) {
    Eigen::Affine3d aff = Eigen::Affine3d::Identity();
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotation, rotationMatrix);
    Eigen::Matrix3d eigenRotationMatrix;
    cv2eigen(rotationMatrix, eigenRotationMatrix);
    aff.prerotate(eigenRotationMatrix);
    aff.pretranslate(Eigen::Vector3d(translation[0]
            ,translation[1],translation[2]));

    Eigen::Matrix4d m = aff.matrix();
    cout<<"Camera affine = "<<m<<endl;

    return m;
}

void printMatrix(Vec3d buf)
{
    cout<<buf[0]*180/3.14 << "," << buf[1]*180/3.14 <<"," <<buf[2]*180/3.14<<endl;
}


