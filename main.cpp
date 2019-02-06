#include <CameraTCPOrientation.h>
#include <RealsensePoseEstimation.h>
#include <Eigen/Geometry>

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
Eigen::affine4d createAffineFromRotationAndTranslation(Vec3d rotation, Vec3d translation);

int main() {

    RealsensePoseEstimation poseEstimation("/home/rob-ot/Documents/calibration/CameraA/camCalibParams.xml", 1.5f);
    auto[rvec, tvec] = poseEstimation.visualizeDetectedAxesAndReturnTransformation();

    CameraTCPOrientation orientation("192.168.1.101");
    if(orientation.initializeModbus()) {
        CameraTCPOrientation::TCP currentTCP = orientation.getTCPOrientation();
        cout<<"x= "<<currentTCP.x<<" y=" <<currentTCP.y<<" z= "<<currentTCP.z<<endl;
        totalTransformation(currentTCP, rvec, tvec);
        delete(&currentTCP);
    }

    return 0;
}

void totalTransformation(CameraTCPOrientation::TCP currentTCP, Vec3d rvec, Vec3d tvec) {
    Mat frameRotationCam, frameRotationBase, netRotation;
    cv::Rodrigues(rvec, frameRotationCam);
    rvec = cv::Vec3d(currentTCP.rx/1000,  currentTCP.ry/1000, currentTCP.rz/1000);
    cv::Rodrigues(rvec, frameRotationBase);

    Eigen::affine4d transformationBase = createAffineFromRotationAndTranslation(rvec, tvec);

    cout<<"Rotation base = "<<frameRotationBase <<" rotationCam=  "<<frameRotationCam<<endl;
    cout<<"Translation base= "<<currentTCP.x<<","<<currentTCP.y<<","<<currentTCP.z<<"  translation cam = ";
    printMatrix(tvec);
}

Eigen::affine4d createAffineFromRotationAndTranslation(Vec3d rotation, Vec3d translation) {
    Eigen::Affine3d rx =
            Eigen::Affine3d(Eigen::AngleAxisd(rotation[0], Eigen::Vector3d(1, 0, 0)));
    Eigen::Affine3d ry =
            Eigen::Affine3d(Eigen::AngleAxisd(rotation[1], Eigen::Vector3d(0, 1, 0)));
    Eigen::Affine3d rz =
            Eigen::Affine3d(Eigen::AngleAxisd(rotation[2], Eigen::Vector3d(0, 0, 1)));
    Eigen::Affine3d rotation = rz * ry * rx;

    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(translation[0]
            ,translation[1],translation[2])));

    Eigen::Matrix4d m = t.matrix();
    m *= r.matrix();
    return m;
}

void printMatrix(Vec3d buf)
{
    cout<<buf[0] << "," << buf[1] <<"," <<buf[2]<<endl;
}

