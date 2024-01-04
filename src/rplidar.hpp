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

namespace viam {
namespace rplidar {

constexpr char kResourceType[] = "CameraRPLiDAR";
constexpr char kAPINamespace[] = "viam";
constexpr char kAPIType[] = "camera";
constexpr char kAPISubtype[] = "rplidar";


struct RPLiDARProperties {
    bool enablePointClouds;
};

// The underlying rplidar loop functions


// Module functions
std::vector<std::string> validate(sdk::ResourceConfig cfg);

// The camera module class and its methods
class RPLiDAR : public sdk::Camera {
   private:
    std::tuple<RPLiDARProperties, bool, bool> initialize(sdk::ResourceConfig cfg);

   public:
    explicit RPLiDAR(sdk::Dependencies deps, sdk::ResourceConfig cfg);
    ~RPLiDAR();
    void reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) override;
    sdk::Camera::raw_image get_image(std::string mime_type,
                                     const sdk::AttributeMap& extra) override;
    sdk::Camera::properties get_properties() override;
    sdk::Camera::image_collection get_images() override;
    sdk::AttributeMap do_command(sdk::AttributeMap command) override;
    sdk::Camera::point_cloud get_point_cloud(std::string mime_type,
                                             const sdk::AttributeMap& extra) override;
    std::vector<sdk::GeometryConfig> get_geometries(const sdk::AttributeMap& extra) override;
};

}  // namespace rplidar
}  // namespace viam