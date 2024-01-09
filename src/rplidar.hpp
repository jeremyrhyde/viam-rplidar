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
constexpr char kAPINamespace[] = "jeremyrhyde";
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

// The underlying rplidar driver functions
bool start_motor(rpsdk::RPlidarDriver *driver);
bool stop_motor(rpsdk::RPlidarDriver *driver);

// Functions that handle scanning
PointXYZI create_point(float dist, float angle);
PointCloudXYZI scan(rpsdk::RPlidarDriver *driver, float min_range_mm);

std::string point_to_string(PointXYZI point);
std::vector<unsigned char> PointCloudXYZI_to_pcd_bytes(PointCloudXYZI point_cloud);

// Math functions
float get_angle(const rplidar_response_measurement_node_hq_t &node);
float get_distance(const rplidar_response_measurement_node_hq_t &node);

// Module functions
std::vector<std::string> validate(sdk::ResourceConfig cfg);

size_t default_node_size = 8192;

// Defaults 
std::string default_serial_port = "/dev/ttyUSB0";
int default_baudrate = 256000;
std::map<std::string, int> baudrate_map{
    {"A1", 256000}, 
    {"A2", 256000}, 
    {"A3", 256000}, 
    {"S1", 256000}, 
    {"S2", 1000000}, 
    {"S3", 1000000},
};

// The camera module class and its methods
class RPLidar : public sdk::Camera {
   private:
    std::string serial_port;
    bool use_caching = false;
    float min_range_mm;
    int serial_baudrate;
    std::string rplidar_model;

    rpsdk::RPlidarDriver *driver = NULL;

    void initialize(sdk::ResourceConfig cfg);
    bool connect();
    bool start();
    
    std::mutex cache_mutex;
    PointCloudXYZI cached_pc;
    std::vector<unsigned char> cached_pcd;

    std::thread cameraThread;
    bool start_polling();
    void scanCacheLoop(std::promise<void>& ready);

    std::mutex reconfiguration_mutex;
    std::atomic<bool> isRunning = false;
    std::atomic<bool> thread_shutdown = false;
    std::condition_variable cv;

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