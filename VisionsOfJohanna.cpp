//
// Created by rob-ot on 1.2.19.
//

#include <librealsense2/rs.hpp>
#include <algorithm>            // std::min, std::max
#include <opencv2/opencv.hpp>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
#include <RealsenseManager.h>

#include <CameraTCPOrientation.h>
#include <RealsensePoseEstimation.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>


void remove_background(rs2::video_frame& color, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
void totalTransformation(CameraTCPOrientation::TCP currentTCP, Vec3d rvec, Vec3d tvec);
void printMatrix(Vec3d buf);
Eigen::Matrix4f createAffineFromRotationAndTranslation(Vec3f rotation, Vec3f translation);


int user_data;
using namespace cv;
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}


// Get RGB values based on normals - texcoords, normals value [u v]
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();

    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}


tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformedPclPoints(const rs2::points &points, const rs2::video_frame &color,
        Eigen::Matrix4f transformer){

    // OpenCV Mat for showing the rgb color image, just as part of processing
    Mat colorr(Size(640, 480), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());




    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); ++i)
    {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color, tex_coords[i]);

        // Reversed order- 2-1-0 because of BGR model used in camera
        cloud->points[i].r = std::get<2>(current_color);
        cloud->points[i].g = std::get<1>(current_color);
        cloud->points[i].b = std::get<0>(current_color);

    }
    pcl::transformPointCloud(*cloud, *transformed_cloud, transformer);

    return make_tuple(transformed_cloud, cloud);
}


void totalTransformation(CameraTCPOrientation::TCP currentTCP, Vec3d rvec, Vec3d tvec) {
    Mat frameRotationCam, frameRotationBase, netRotation;
    cv::Rodrigues(rvec, frameRotationCam);
    rvec = cv::Vec3d(currentTCP.rx/1000,  currentTCP.ry/1000, currentTCP.rz/1000);
    cv::Rodrigues(rvec, frameRotationBase);

    Eigen::Matrix4f transformationBase = createAffineFromRotationAndTranslation(rvec, tvec);

    cout<<"Rotation base = "<<frameRotationBase <<" rotationCam=  "<<frameRotationCam<<endl;
    cout<<"Translation base= "<<currentTCP.x<<","<<currentTCP.y<<","<<currentTCP.z<<"  translation cam = ";
    printMatrix(tvec);
}

Eigen::Matrix4f createAffineFromRotationAndTranslation(Vec3f rotation, Vec3f translation) {
    Eigen::Affine3f rx =
            Eigen::Affine3f(Eigen::AngleAxisf(rotation[0], Eigen::Vector3f(1, 0, 0)));
    Eigen::Affine3f ry =
            Eigen::Affine3f(Eigen::AngleAxisf(rotation[1], Eigen::Vector3f(0, 1, 0)));
    Eigen::Affine3f rz =
            Eigen::Affine3f(Eigen::AngleAxisf(rotation[2], Eigen::Vector3f(0, 0, 1)));
    Eigen::Affine3f r = rz * ry * rx;

    Eigen::Affine3f t(Eigen::Translation3f(Eigen::Vector3f(translation[0]
            ,translation[1],translation[2])));

    Eigen::Matrix4f m = t.matrix();
    m *= r.matrix();
    return m;
}

void printMatrix(Vec3d buf)
{
    cout<<buf[0] << "," << buf[1] <<"," <<buf[2]<<endl;
}



int main() {

    RealsensePoseEstimation poseEstimation("/home/rob-ot/Documents/calibration/CameraA/camCalibParams.xml", 1.5f);
    auto[rvec, tvec] = poseEstimation.visualizeDetectedAxesAndReturnTransformation();

    //CameraTCPOrientation orientation("192.168.1.101");
    /*if(orientation.initializeModbus()) {
        CameraTCPOrientation::TCP currentTCP = orientation.getTCPOrientation();
        cout<<"x= "<<currentTCP.x<<" y=" <<currentTCP.y<<" z= "<<currentTCP.z<<endl;
        totalTransformation(currentTCP, rvec, tvec);
    }*/

    Eigen::Matrix4f transformer1 = createAffineFromRotationAndTranslation(rvec, tvec);

    rs2::pointcloud pc;

    RealsenseManager manager = poseEstimation.getDeviceManager();
    pcl::visualization::CloudViewer viewer ("Visions of Johanna");
    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);

    while (!viewer.wasStopped ())
    {
        auto[image, depth_information, video_framed] =  manager.getCVAlignedMatrix();


        // Order here is crucial!
        // map_to() color frame has to be done befor point calculation
        // otherwise texture won't be mapped
        pc.map_to(video_framed);
        auto points = pc.calculate(depth_information);

        // Actual calling of conversion and saving XYZRGB cloud to file
        auto[transformed_cloud, cloud] =
                transformedPclPoints(points, video_framed, transformer1);


        viewer.showCloud(transformed_cloud);

        user_data++;
    }

    return EXIT_SUCCESS;
}