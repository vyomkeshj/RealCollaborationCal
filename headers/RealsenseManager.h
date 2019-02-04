#pragma once
//
// Created by rob-ot on 18.1.19.
//
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace rs2;

class RealsenseManager {
public:


    RealsenseManager(float depthFilter) {
        this->depthFilter = depthFilter;
        cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        profile = pipe.start(cfg);
        depth_scale = get_depth_scale(profile.get_device());
        align_to = find_stream_to_align(profile.get_streams());
        align = new rs2::align(align_to);
    }

    ~RealsenseManager() {
        if (align != nullptr)
            delete align;
    }

    void prepareCamera() {
        for (int i = 1; i <30; i++) {
            frameset = pipe.wait_for_frames();
        }
    }

    float getDepthScale() {
        return depth_scale;
    }

    tuple<Mat, depth_frame> getCVAlignedMatrix() {
    bool received_frames = false;
    while(!received_frames) {
        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams())) {
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            if (align != nullptr)
                delete align;
            align = new rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }
        frameset = pipe.wait_for_frames();
        //get the processed aligned frame
        auto processed = align->process(frameset);

        // Trying to get both color and aligned depth frames
        rs2::video_frame other_frame = processed.first_or_default(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();
        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame) {
            continue;
        } else {
            received_frames = true;
            remove_background(other_frame, aligned_depth_frame, depth_scale, depthFilter);
            // Creating OpenCV matrix from IR image
            Mat ir(Size(640, 480), CV_8UC1, (void *) other_frame.get_data(), Mat::AUTO_STEP);
            return make_tuple(ir, aligned_depth_frame);
        }
    }
    }

    depth_frame getAlignedDepthFrame() {

    };
private:
    pipeline pipe;
    config cfg;
    pipeline_profile profile;
    rs2_stream align_to;
    rs2::align *align = nullptr;
    rs2::frameset frameset;

    int selectedCamera = 0;
    int numberOfAvailableCameras = 0;

    float depth_scale;
    float depthFilter = 0.0f;


    void remove_background(video_frame& other_frame, const depth_frame& depth_frame, float depth_scale, float clipping_dist)
    {
        const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
        uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

        int width = other_frame.get_width();
        int height = other_frame.get_height();
        int other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic)
        for (int y = 0; y < height; y++)
        {
            auto depth_pixel_index = y * width;
            for (int x = 0; x < width; x++, ++depth_pixel_index)
            {
                // Get the depth value of the current pixel
                auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

                // Check if the depth value is invalid (<=0) or greater than the threashold
                if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
                {
                    // Calculate the offset in other frame's buffer to current pixel
                    auto offset = depth_pixel_index * other_bpp;

                    // Set pixel to "background" color (0x999999)
                    std::memset(&p_other_frame[offset], 0x99, other_bpp);
                }
            }
        }
    }

    rs2_stream find_stream_to_align(const vector<stream_profile>& streams)
    {
        rs2_stream align_to = RS2_STREAM_ANY;
        bool depth_stream_found = false;
        bool color_stream_found = false;
        for (rs2::stream_profile sp : streams)
        {
            rs2_stream profile_stream = sp.stream_type();
            if (profile_stream != RS2_STREAM_DEPTH)
            {
                if (!color_stream_found)         //Prefer color
                    align_to = profile_stream;

                if (profile_stream == RS2_STREAM_COLOR)
                {
                    color_stream_found = true;
                }
            }
            else
            {
                depth_stream_found = true;
            }
        }

        if(!depth_stream_found)
            throw std::runtime_error("No Depth stream available");

        if (align_to == RS2_STREAM_ANY)
            throw std::runtime_error("No stream found to align with Depth");

        return align_to;
    }

    float get_depth_scale(rs2::device dev)
    {
        // Go over the device's sensors
        for (rs2::sensor& sensor : dev.query_sensors())
        {
            // Check if the sensor if a depth sensor
            if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
            {
                return dpt.get_depth_scale();
            }
        }
        throw std::runtime_error("Device does not have a depth sensor");
    }

    bool profile_changed(const vector<stream_profile>& current, const vector<stream_profile>& prev)
    {
        for (auto&& sp : prev)
        {
            auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
            if (itr == std::end(current))
            {
                return true;
            }
        }
        return false;
    }
};