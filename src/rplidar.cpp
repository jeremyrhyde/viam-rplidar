#include "rplidar.hpp"

#include <arpa/inet.h>
#include <pthread.h>
#include <signal.h>

#include <algorithm>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>

#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>

// #include "third_party/fpng.h"
// #include "third_party/lodepng.h"

namespace viam {
namespace rplidar {

// initialize will use the ResourceConfigs to begin the rplidar pipeline.
std::tuple<RPLidarProperties, bool, bool> RPLidar::initialize(sdk::ResourceConfig cfg) {
    return {};
}

// CAMERA module methods
RPLidar::RPLidar(sdk::Dependencies deps, sdk::ResourceConfig cfg) : Camera(cfg.name()) {


//   drv = RPlidarDriver::CreateDriver(
//       rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

//   if (!drv) {
//     throw std::runtime_error("failed to create RPLidar driver.");
//   }

//   if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
//     RPlidarDriver::DisposeDriver(drv);
//     throw std::runtime_error(
//         "Error: Cannot bind RPLidar to the specified serial port " +
//         serial_port + ".");
//   }

//   if (!getDeviceInfo()) {
//     RPlidarDriver::DisposeDriver(drv);
//     throw std::runtime_error("failed to get device info");
//   }
  
}

RPLidar::~RPLidar() {
}

// Reconfigure
void RPLidar::reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) {
    std::cerr << "reconfigure not implemented" << std::endl;
    return;
}

// GetImage
sdk::Camera::raw_image RPLidar::get_image(std::string mime_type, const sdk::AttributeMap& extra) {
    std::cerr << "get_image not implemented" << std::endl;
    return {};
}

// GetImages
sdk::Camera::image_collection RPLidar::get_images() {
    std::cerr << "get_images not implemented" << std::endl;
    return {};
}

// GetPointCloud
sdk::Camera::point_cloud RPLidar::get_point_cloud(std::string mime_type, const sdk::AttributeMap& extra) {
    sdk::Camera::point_cloud point_cloud;
    point_cloud.mime_type = "pointcloud/pcd";
    std::vector<unsigned char> bytes = {'a', 'b', 'c'};
    point_cloud.pc = bytes;
    std::cerr << "get_point_cloud not implemented" << std::endl;
    return point_cloud;
}

// GetGeometries
std::vector<sdk::GeometryConfig> RPLidar::get_geometries(const sdk::AttributeMap& extra) {
    std::cerr << "get_geometries not implemented" << std::endl;
    return std::vector<sdk::GeometryConfig>{};
}

// GetProperties
sdk::Camera::properties RPLidar::get_properties() {
    std::cerr << "get_properties not implemented" << std::endl;
    struct sdk::Camera::properties prop;
    prop.supports_pcd = true;

    return prop;
}

// DoCommand
sdk::AttributeMap RPLidar::do_command(sdk::AttributeMap command) {
    std::cerr << "do_command not implemented" << std::endl;
    return sdk::AttributeMap{};
}

}  // namespace rplidar
}  // namespace viam