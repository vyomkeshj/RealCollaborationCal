//
// Created by rob-ot on 6.2.19.
//

#ifndef REALCOLLABORATIONCAL_REALSENSEDEVICEPROVIDER_H
#define REALCOLLABORATIONCAL_REALSENSEDEVICEPROVIDER_H

#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <mutex>
#include <map>

/*
 * @Class Realsense Viewer is the base layer to communicate with the realsense cameras
 *  it allows basic features like enabling multiple cameras, keep their list, polling of frames and retrieval of camera data.
 * **/
using namespace std;
const std::string platform_camera_name = "Platform Camera";
class RealsenseDeviceProvider {

    //A structure representing each device

public:
    /**
     * This structure stores the information about the activated cameras
     * */
    struct view_port
    {
        rs2::frameset current_frameset;  //Current Frameset as from the camera, is refreshed in this structure by calling pollFrames();
        rs2::pipeline pipe;
        rs2::pipeline_profile profile;
    };

    /*
     * Enables the device provided as {@Param dev}, puts it into a list of view_port
     * **/
    void enableDevice(rs2::device dev)
    {
        std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        std::lock_guard<std::mutex> lock(_mutex);

        if (_devices.find(serial_number) != _devices.end())
        {
            return; //already in
        }

        // Ignoring platform cameras (webcams, etc..)
        if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
        {
            return;
        }
        // Create a pipeline from the given device
        rs2::pipeline p;
        rs2::config c;                                                      //configuration of the camera
        c.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);  //choose the resolution and fps
        c.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30); //Cannot be a random combination, please refer
        c.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);//realsense documentation for available configurations.
        //c.enable_all_streams();

        c.enable_device(serial_number);

        // Start the pipeline with the configuration
        try {
            rs2::pipeline_profile profile = p.start(c);     //start pipe p with camera configutation c
            _devices.emplace(serial_number, view_port{ {}, p, profile });

        } catch (const rs2::error & e)
        {
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        }

        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        // Hold it internally
    }
    /*
     * Removes the device from the list of enabled devices
     * **/
    void removeDevices(const rs2::event_information& info)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // Go over the list of devices and check if it was disconnected
        auto itr = _devices.begin();
        while(itr != _devices.end())
        {
            if (info.was_removed(itr->second.profile.get_device()))
            {
                itr = _devices.erase(itr);
            }
            else
            {
                ++itr;
            }
        }
    }
    /*
     * Check all cameras for new frames, update their view port with the new frames
     * **/
    void pollFrames()
    {
        try {
            std::lock_guard<std::mutex> lock(_mutex);
            // Go over all device
            for (auto &&view : _devices) {
                // Ask each pipeline if there are new frames available
                rs2::frameset frameset;
                if (view.second.pipe.poll_for_frames(&frameset)) {
                    view.second.current_frameset = frameset; //update view port with the new frameset
                }
            }
        } catch (rs2::error &e) {
            std::cout<<"realsense frame capture failed";
        }
    }

    /**
     * Returns the number of devices in the enabled devices list
     * */
    int deviceCount()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _devices.size();
    }

    /***
     * returns the list of enabled cameras
     * @return _devices, the list of current devices
     */
    std::map<std::string, view_port>& getEnabledDevices() {
        return _devices;
    }

    rs2::context& getContext() {
        return context;
    }

private:
    std::mutex _mutex;
    std::map<std::string, view_port> _devices;  //map with camera's serial number and view port
    rs2::context context;
};


#endif //REALCOLLABORATIONCAL_REALSENSEDEVICEPROVIDER_H
