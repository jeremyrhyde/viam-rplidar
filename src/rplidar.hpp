#pragma once
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

#include <viam/sdk/components/camera/camera.hpp>
#include <viam/sdk/components/camera/server.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>

// #include "./thrid_party/rplidar_sdk/sdk/include/rplidar.h"
// #include "./thrid_party/rplidar_sdk/sdk/src/hal/types.h"
// #include "./thrid_party/rplidar_sdk/sdk/include/rplidar_protocol.h"
// #include "./thrid_party/rplidar_sdk/sdk/include/rplidar_cmd.h"
// #include "./thrid_party/rplidar_sdk/sdk/include/rplidar_driver.h"

// /sl_lidar_driver.h"
// %include "./third_party/rplidar_sdk-release-v2.0.0/sdk/include/rplidar.h"
// %include "./third_party/rplidar_sdk-release-v2.0.0/sdk/src/hal/types.h"
// %include "./third_party/rplidar_sdk-release-v2.0.0/sdk/include/rplidar_protocol.h"
// %include "./third_party/rplidar_sdk-release-v2.0.0/sdk/include/rplidar_cmd.h"
// %include "./third_party/rplidar_sdk-release-v2.0.0/sdk/include/rplidar_driver.h"

// class rplidar_response_measurement_node_hq_t;
// namespace rp {
//   namespace standalone {
//     namespace rplidar {
//       class RPlidarDriver;
//     }
//   }
// }

namespace viam {
namespace rplidar {

constexpr char kResourceType[] = "RPLidar";
constexpr char kAPINamespace[] = "viam";
constexpr char kAPIType[] = "camera";
constexpr char kAPISubtype[] = "rplidar";


struct RPLidarProperties {
    bool enablePointClouds;
};

// The underlying rplidar loop functions


// Module functions
std::vector<std::string> validate(sdk::ResourceConfig cfg);

// The camera module class and its methods
class RPLidar : public sdk::Camera {
   private:
    std::string serial_port = "";
    int serial_baudrate;
    std::string scan_mode = "";
    // rp::standalone::rplidar::RPlidarDriver *driver = NULL;

    std::tuple<RPLidarProperties, bool, bool> initialize(sdk::ResourceConfig cfg);

   public:
    explicit RPLidar(sdk::Dependencies deps, sdk::ResourceConfig cfg);
    ~RPLidar();

    void reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) override;

    sdk::Camera::raw_image get_image(std::string mime_type, const sdk::AttributeMap& extra) override;
    sdk::Camera::image_collection get_images() override;
    sdk::Camera::point_cloud get_point_cloud(std::string mime_type, const sdk::AttributeMap& extra) override;
    std::vector<sdk::GeometryConfig> get_geometries(const sdk::AttributeMap& extra) override;
    sdk::Camera::properties get_properties() override;
    sdk::AttributeMap do_command(sdk::AttributeMap command) override;
};

}  // namespace rplidar
}  // namespace viam