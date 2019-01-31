#include <CameraTCPOrientation.h>
#include <RealsensePoseEstimation.h>

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
    }

    return 0;
}

void totalTransformation(CameraTCPOrientation::TCP currentTCP, Vec3d rvec, Vec3d tvec) {
    Vec3d finalRVec, finalTVec;
    Mat frameRotationCam, frameRotationBase, netRotation;
    cv::Rodrigues(rvec, frameRotationCam);
    rvec = cv::Vec3d(currentTCP.rx/1000,  currentTCP.ry/1000, currentTCP.rz/1000);
    cv::Rodrigues(rvec, frameRotationBase);


    finalTVec[0] = currentTCP.x/10000 + tvec[0];
    finalTVec[1] = currentTCP.y/10000 + tvec[1];
    finalTVec[2] = currentTCP.z/10000 + tvec[2];
    cout<<endl<<"RVEC: ";
    printMatrix(rvec);
    cout<<"TVEC: ";
    printMatrix(finalTVec);
}

void printMatrix(Vec3d buf)
{
    cout<<buf[0] << "," << buf[1] <<"," <<buf[2]<<endl;
}

