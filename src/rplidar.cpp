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
std::tuple<RPLiDARProperties, bool, bool> RPLiDAR::initialize(sdk::ResourceConfig cfg) {
    return {};
}

// CAMERA module methods
RPLiDAR::RPLiDAR(sdk::Dependencies deps, sdk::ResourceConfig cfg) : Camera(cfg.name()) {
}

RPLiDAR::~RPLiDAR() {
}

// Reconfigure
void RPLiDAR::reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) {
    std::cerr << "reconfigure not implemented" << std::endl;
    return;
}

// GetImage
sdk::Camera::raw_image RPLiDAR::get_image(std::string mime_type, const sdk::AttributeMap& extra) {
    std::cerr << "get_image not implemented" << std::endl;
    return {};
}

// GetProperties
sdk::Camera::properties RPLiDAR::get_properties() {
    std::cerr << "get_properties not implemented" << std::endl;
    struct sdk::Camera::properties prop;
    prop.supports_pcd = true;

    return prop;
}

// GetImages
sdk::Camera::image_collection RPLiDAR::get_images() {
    std::cerr << "get_images not implemented" << std::endl;
    return {};
}

// DoCommand
sdk::AttributeMap RPLiDAR::do_command(sdk::AttributeMap command) {
    std::cerr << "do_command not implemented" << std::endl;
    return sdk::AttributeMap{};
}

// GetPointCloud
sdk::Camera::point_cloud RPLiDAR::get_point_cloud(std::string mime_type, const sdk::AttributeMap& extra) {
    std::cerr << "get_point_cloud not implemented" << std::endl;
    return sdk::Camera::point_cloud{};
}

// GetGeometries
std::vector<sdk::GeometryConfig> RPLiDAR::get_geometries(const sdk::AttributeMap& extra) {
    std::cerr << "get_geometries not implemented" << std::endl;
    return std::vector<sdk::GeometryConfig>{};
}

}  // namespace rplidar
}  // namespace viam