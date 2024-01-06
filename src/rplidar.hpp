#pragma once

#include <future>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>

#include <viam/sdk/components/camera/camera.hpp>
#include <viam/sdk/components/camera/server.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>

#include "rplidar.h"

namespace viam {
namespace rplidar {

namespace rpsdk = rp::standalone::rplidar;

// Viam resource naming constants 
constexpr char kResourceType[] = "RPLidar";
constexpr char kAPINamespace[] = "viam";
constexpr char kAPIType[] = "camera";
constexpr char kAPISubtype[] = "rplidar";

// Internal pointcloud and rplidar structs
struct PointXYZI {
    float x;
    float y;
    float z;
    float intensity;
};

struct PointCloudXYZI {
    int count;
    std::vector<PointXYZI> points;
};

struct RPLidarProperties {
    bool enablePointClouds;
};

// The underlying rplidar driver functions
bool start_motor(rpsdk::RPlidarDriver *driver);
bool stop_motor(rpsdk::RPlidarDriver *driver);

// Functions that handle scanning
PointXYZI create_point(float dist, float angle, float intensity);
PointCloudXYZI scan(rpsdk::RPlidarDriver *driver, float min_range_mm);

// Math functions
float get_angle(const rplidar_response_measurement_node_hq_t &node);
float get_distance(const rplidar_response_measurement_node_hq_t &node);
float get_quality(const rplidar_response_measurement_node_hq_t &node);

// Module functions
std::vector<std::string> validate(sdk::ResourceConfig cfg);

size_t default_node_size = 8192;

// The camera module class and its methods
class RPLidar : public sdk::Camera {
   private:
    rpsdk::RPlidarDriver *driver = NULL;
    std::string serial_port = "/dev/ttyUSB0";
    float min_range_mm = 0.0;
    int serial_baudrate = 10000000;
    std::string rplidar_model = "";
    std::string scan_mode = "";

    std::tuple<RPLidarProperties, bool, bool> initialize(sdk::ResourceConfig cfg);

    bool connect();
    bool start();

    bool start_polling();
    void scanCacheLoop(std::promise<void>& ready);
    std::atomic<bool> thread_shutdown = false;
    std::thread cameraThread;
    
    std::mutex cache_mutex;
    PointCloudXYZI cached_pc;

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