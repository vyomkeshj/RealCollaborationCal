//
// Created by rob-ot on 6.2.19.
//

#ifndef REALCOLLABORATIONCAL_REALSENSEDEVICEPROVIDER_H
#define REALCOLLABORATIONCAL_REALSENSEDEVICEPROVIDER_H

#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <mutex>
#include <map>

using namespace std;
const std::string platform_camera_name = "Platform Camera";
class RealsenseDeviceProvider {

    //A structure representing each device
    struct view_port
    {
        rs2::frameset current_frameset;
        rs2::pipeline pipe;
        rs2::pipeline_profile profile;
    };

public:
    void enable_device(rs2::device dev)
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
        rs2::config c;
        c.enable_device(serial_number);
        // Start the pipeline with the configuration
        rs2::pipeline_profile profile = p.start(c);
        // Hold it internally
        _devices.emplace(serial_number, view_port{ {}, p, profile });

    }

    void remove_devices(const rs2::event_information& info)
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

    void poll_frames()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // Go over all device
        for (auto&& view : _devices)
        {
            // Ask each pipeline if there are new frames available
            rs2::frameset frameset;
            if (view.second.pipe.poll_for_frames(&frameset))
            {
                    view.second.current_frameset = frameset; //update view port with the new stream
            }
        }
    }
    size_t device_count()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _devices.size();
    }

    std::map<std::string, view_port> getEnabledDevices() {
        return _devices;
    }

private:
    std::mutex _mutex;
    std::map<std::string, view_port> _devices;
};


#endif //REALCOLLABORATIONCAL_REALSENSEDEVICEPROVIDER_H
