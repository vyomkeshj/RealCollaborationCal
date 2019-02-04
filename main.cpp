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

    cout<<"Rotation base = "<<frameRotationBase <<" rotationCam=  "<<frameRotationCam<<endl;
    cout<<"Translation base= "<<currentTCP.x<<","<<currentTCP.y<<","<<currentTCP.z<<"  translation cam = ";
    printMatrix(tvec);



}

void printMatrix(Vec3d buf)
{
    cout<<buf[0] << "," << buf[1] <<"," <<buf[2]<<endl;
}

