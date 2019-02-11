#pragma once
//
// Created by rob-ot on 18.1.19.
//
#define DEVICE_CALIB_CAM 0
#define DEVICE_REGULAR_CAM 1

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "RealsenseDeviceProvider.h"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;
using namespace cv;
using namespace rs2;

class RealsenseManager {
public:


    RealsenseManager(float depthFilter) {
        this->depthFilter = depthFilter;
        initializeCameras();

    }

    RealsenseManager() {
        initializeCameras();
    }
    ~RealsenseManager() {
    }

    float getDepthScale() {
        return depth_scale;
    }



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudFromCamera(const string &cameraSerial) {
        RealsenseDeviceProvider::view_port currentViewPort = getCameraStream(cameraSerial);
        rs2::frameset currentFrameset = currentViewPort.current_frameset;

        rs2::video_frame currentVideoFrame = currentFrameset.first(RS2_STREAM_COLOR);
        rs2::video_frame currentDepthFrame = currentFrameset.first(RS2_STREAM_DEPTH);

        rs2::pointcloud pc;
        pc.map_to(currentVideoFrame);
        auto points = pc.calculate(currentDepthFrame);
        return convertToPclCloud(points, currentVideoFrame);
    }

    void grabNewFrames() {
        provider.pollFrames();
    }

    std::vector<string> getConnectedDeviceIds() {
        std::vector<string> deviceIds;
        std::map<string, RealsenseDeviceProvider::view_port> deviceMap = provider.getEnabledDevices();
        std::transform(deviceMap.begin(), deviceMap.end(), back_inserter(deviceIds), RetrieveKey());
        return deviceIds;
    }

    void initializeCameras() {
        rs2::context ctx = provider.getContext();
        ctx.set_devices_changed_callback([&](rs2::event_information &info) {
            provider.removeDevices(info);
            for (auto &&dev : info.get_new_devices()) {
                provider.enableDevice(dev);
            }
            std::cout<<"number of connected RealsenseDevices = "<<getNumberOfDevices()<<std::endl;
        });

        // Initial population of the device list
        for (auto &&dev : ctx.query_devices()) // Query the list of connected RealSense devices
        {
            provider.enableDevice(dev);
        }
        std::cout<<"number of connected RealsenseDevices = "<<getNumberOfDevices()<<std::endl;

        for (int i = 0; i < 100; i++) {
            grabNewFrames();
        }
    }

    int getNumberOfDevices() {
        return provider.deviceCount();
    }

    RealsenseDeviceProvider::view_port getCameraStream(const string &cameraSerial) {
        return provider.getEnabledDevices().at(cameraSerial);
    }

private:
    RealsenseDeviceProvider provider;

    int selectedCamera = 0;
    int numberOfAvailableCameras = 0;

    float depth_scale;
    float depthFilter = 0.0f;


    // Get RGB values based on normals - texcoords, normals value [u v]
    std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords) {
        const int w = texture.get_width(), h = texture.get_height();

        // convert normals [u v] to basic coords [x y]
        int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
        int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);

        int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
        const auto texture_data = reinterpret_cast<const uint8_t *>(texture.get_data());
        return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
    }

    void remove_background(video_frame &other_frame, const depth_frame &depth_frame, float depth_scale,
                           float clipping_dist) {
        const uint16_t *p_depth_frame = reinterpret_cast<const uint16_t *>(depth_frame.get_data());
        uint8_t *p_other_frame = reinterpret_cast<uint8_t *>(const_cast<void *>(other_frame.get_data()));

        int width = other_frame.get_width();
        int height = other_frame.get_height();
        int other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic)
        for (int y = 0; y < height; y++) {
            auto depth_pixel_index = y * width;
            for (int x = 0; x < width; x++, ++depth_pixel_index) {
                // Get the depth value of the current pixel
                auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

                // Check if the depth value is invalid (<=0) or greater than the threashold
                if (pixels_distance <= 0.f || pixels_distance > clipping_dist) {
                    // Calculate the offset in other frame's buffer to current pixel
                    auto offset = depth_pixel_index * other_bpp;

                    // Set pixel to "background" color (0x999999)
                    std::memset(&p_other_frame[offset], 0x99, other_bpp);
                }
            }
        }
    }

    rs2_stream find_stream_to_align(const vector<stream_profile> &streams) {
        rs2_stream align_to = RS2_STREAM_ANY;
        bool depth_stream_found = false;
        bool color_stream_found = false;
        for (rs2::stream_profile sp : streams) {
            rs2_stream profile_stream = sp.stream_type();
            if (profile_stream != RS2_STREAM_DEPTH) {
                if (!color_stream_found)         //Prefer color
                    align_to = profile_stream;

                if (profile_stream == RS2_STREAM_COLOR) {
                    color_stream_found = true;
                }
            } else {
                depth_stream_found = true;
            }
        }

        if (!depth_stream_found)
            throw std::runtime_error("No Depth stream available");

        if (align_to == RS2_STREAM_ANY)
            throw std::runtime_error("No stream found to align with Depth");

        return align_to;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPclCloud(const rs2::points &points, const rs2::video_frame &color) {

        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Config of PCL Cloud object
        cloud->width = static_cast<uint32_t>(sp.width());
        cloud->height = static_cast<uint32_t>(sp.height());
        cloud->is_dense = false;
        cloud->points.resize(points.size());


        auto tex_coords = points.get_texture_coordinates();
        auto vertices = points.get_vertices();

        // Iterating through all points and setting XYZ coordinates
        // and RGB values
        for (int i = 0; i < points.size(); ++i) {
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

        return cloud;
    }

    float get_depth_scale(rs2::device dev) {
        // Go over the device's sensors
        for (rs2::sensor &sensor : dev.query_sensors()) {
            // Check if the sensor if a depth sensor
            if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
                return dpt.get_depth_scale();
            }
        }
        throw std::runtime_error("Device does not have a depth sensor");
    }

    bool profile_changed(const vector<stream_profile> &current, const vector<stream_profile> &prev) {
        for (auto &&sp : prev) {
            auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const stream_profile &current_sp) {
                return sp.unique_id() == current_sp.unique_id();
            });
            if (itr == std::end(current)) {
                return true;
            }
        }
        return false;
    }


    struct RetrieveKey {
        template<typename T>
        typename T::first_type operator()(T keyValuePair) const {
            return keyValuePair.first;
        }
    };
};