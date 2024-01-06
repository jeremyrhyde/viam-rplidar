#include "rplidar.hpp"

#include <pthread.h>
#include <signal.h>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>
#include <cmath>

#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>


namespace viam {
namespace rplidar {

namespace rpsdk = rp::standalone::rplidar;

std::tuple<RPLidarProperties, bool, bool> RPLidar::initialize(sdk::ResourceConfig cfg) {
    return {};
}

// Motor interface functions : (start, stop)
bool stop_motor(rp::standalone::rplidar::RPlidarDriver *driver) {
  if (!driver) {
    return false;
  }
  driver->stop();
  driver->stopMotor();
  return true;
}

bool start_motor(rp::standalone::rplidar::RPlidarDriver *driver) {
  if (!driver) {
    return false;
  }
  driver->startMotor();
  driver->startScan(false, true);
  return true;
}

// Node interface functions : (angle, dist, quality)
float get_angle(const rplidar_response_measurement_node_hq_t &node) {
  return node.angle_z_q14 * 90.f / (1 << 14);
}

float get_distance(const rplidar_response_measurement_node_hq_t &node) {
  return node.dist_mm_q2 / 4.0f / 1000; 
}

float get_quality(const rplidar_response_measurement_node_hq_t &node) {
  return node.quality >> 2;
}

// Scan and helper functions to handle pointcloud construction and pcd creation
// : (create_point, PointCloudXYZI_to_pcd_bytes, scan) 
PointXYZI create_point(float dist, float angle, float intensity) {
    PointXYZI point;
    point.x = dist*cos(angle * M_PI / 180);
    point.y = dist*sin(angle * M_PI / 180);
    point.z = 0.0;
    point.intensity = intensity;

    return point;
}

std::vector<unsigned char> PointCloudXYZI_to_pcd_bytes(PointCloudXYZI point_cloud) {

    // MAKE PCD BYTE VECTOR SLICE

    return {};
}

PointCloudXYZI scan(rpsdk::RPlidarDriver *driver, float min_range_mm) {
    // Get scan nodes
    size_t count = default_node_size * 1;
    rplidar_response_measurement_node_hq_t nodes[count * sizeof(rplidar_response_measurement_node_hq_t)];

    if (driver->grabScanDataHq(nodes, count) != RESULT_OK) {
        std::cerr << "grabScanDataHq failed to grab data" << std::endl;
        return {};
    }

    if (driver->ascendScanData(nodes, count) != RESULT_OK) {
        std::cerr << "ascendScanData failed to sort data" << std::endl;
        return {};
    }

    // Loop over pointcloud
    PointCloudXYZI pc;
    for (int i = 0; i < count; ++i) {
        float dist_value = get_distance(nodes[i]);
        if (dist_value < min_range_mm) {
            continue;
        }
        float angle_value = get_angle(nodes[i]);
        float intensity_value = get_quality(nodes[i]);

        pc.points.push_back(create_point(dist_value, angle_value, intensity_value));
        pc.count++;
    }

    return pc;
}

// RPLidar Constructor and Destructor : (RPLidar, ~RPLidar)
RPLidar::RPLidar(sdk::Dependencies deps, viam::sdk::ResourceConfig cfg) : Camera(cfg.name()) {

    // Attribute extraction
    auto attrs = cfg.attributes();

    if (attrs->count("serial_path") == 1) {
        std::shared_ptr<sdk::ProtoType> serial_path_proto = attrs->at("serial_path");
        auto serial_path_value = serial_path_proto->proto_value();
        if (serial_path_value.has_string_value()) {
            std::string serial_port_str = static_cast<std::string>(serial_path_value.string_value());
            serial_port = serial_port_str;
            std::cout << "serial_path found, using " << serial_port << std::endl;
        }
    } else {
        std::cout << "no serial_path given, using default " << serial_port << std::endl;
    }

    if (attrs->count("min_range_mm") == 1) {
        std::shared_ptr<sdk::ProtoType> min_range_mm_proto = attrs->at("min_range_mm");
        auto min_range_mm_value = min_range_mm_proto->proto_value();
        if (min_range_mm_value.has_number_value()) {
            float min_range_mm_num = static_cast<float>(min_range_mm_value.number_value());
            min_range_mm = min_range_mm_num;
        }
    }

    // Create the driver
    driver = rpsdk::RPlidarDriver::CreateDriver(rpsdk::DRIVER_TYPE_SERIALPORT);
    if (!driver) {
        throw std::runtime_error("failed to create RPLidar driver");
    } else {
        std::cout << "created RPLidar" << std::endl;
    }

    // Connect to the driver
    if (!connect()) {
        rpsdk::RPlidarDriver::DisposeDriver(driver);
        throw std::runtime_error("could not find RPLidar at specificed serial_path: " + serial_port);
    } else {
        std::cout << "found RPLidar (" << rplidar_model << ") at serial_path " << serial_port << " [Baudrate: " << serial_baudrate << "]" << std::endl;
    }

    // Start the driver
    if (!start()) {
        rpsdk::RPlidarDriver::DisposeDriver(driver);
        throw std::runtime_error("could not start RPLidar");
    } else {
        std::cout << "started RPLidar... " << std::endl;
    }

    // Start background polling process
    // if (!start_polling()) {
    //     rpsdk::RPlidarDriver::DisposeDriver(driver);
    //     throw std::runtime_error("could not start background polling process");
    // } else {
    //     std::cout << "started polling process... " << std::endl;
    // }
}

RPLidar::~RPLidar() {
    thread_shutdown = true;
    cameraThread.join();

    if (!stop_motor(driver)) {
        std::cerr << "issue occured stopping the motor" << std::endl;
    }
    rpsdk::RPlidarDriver::DisposeDriver(driver);
}

// Start up functions : (start, connect, scanCacheLoop, start_polling)
bool RPLidar::start() {
    if (!start_motor(driver)) {
         std::cerr << "issue occurred starting motor" << std::endl;
         return false;
    }
    return true;
}

bool RPLidar::connect() {
    // connect
    if (IS_FAIL(driver->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
        rp::standalone::rplidar::RPlidarDriver::DisposeDriver(driver);
        
    }

    u_result op_result;

    // Check device info
    rplidar_response_device_info_t devinfo;
    op_result = driver->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            std::cerr << "operation timed out" << std::endl;
        } else {
            std::cerr << "unexpected error, code: " << std::hex << static_cast<int>(op_result) << std::endl;
        }
        return false;
    }

    rplidar_model = devinfo.model;

    // Check device health
    rplidar_response_device_health_t healthinfo;
    op_result = driver->getHealth(healthinfo);
    if (IS_FAIL(op_result)) {
        std::cerr << "bad health status received" << std::endl;
        return false;
    } 

    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
        std::cerr << "internal error detected, please reboot the device to retry" << std::endl;
        return false;
    }
    
    return true;
}

void RPLidar::scanCacheLoop(std::promise<void>& ready) {
    bool readyOnce = false;

    while (true) {
        if (thread_shutdown) {
            return;
        }
        PointCloudXYZI pc = scan(driver, min_range_mm);

        cache_mutex.lock();
        cached_pc = pc;
        cache_mutex.unlock();

        if (!readyOnce) {
            readyOnce = true;
            ready.set_value();
        }
    }

    return;
}
 
bool RPLidar::start_polling() {
    std::promise<void> ready;
    cameraThread = std::thread(&RPLidar::scanCacheLoop, this, ref(ready));

    std::cout << "waiting for camera frame loop thread to be ready..." << std::endl;

    ready.get_future().wait();

    return true;
}

// Camera Methods

// Reconfigure
void RPLidar::reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) {
    std::cerr << "reconfigure not implemented" << std::endl;
    return;
}

// GetPointCloud
sdk::Camera::point_cloud RPLidar::get_point_cloud(std::string mime_type, const sdk::AttributeMap& extra) {

    PointCloudXYZI pc = scan(driver, 1);
    if (sizeof(pc)) {
        throw std::runtime_error("error no point_cloud data returned from scan");
    }

    // cache_mutex.lock();
    // if (!cached_pc.count != 0) {
    //     throw std::runtime_error("no point_cloud data is avialable");
    // }
    // PointCloudXYZI pc = cached_pc;
    // cache_mutex.unlock();

    // create point_cloud return message
    sdk::Camera::point_cloud point_cloud;
    point_cloud.mime_type = "pointcloud/pcd";
    point_cloud.pc = PointCloudXYZI_to_pcd_bytes(pc);

    std::cout << "returning pointcloud" << std::endl;
    return point_cloud;
}

// GetProperties
sdk::Camera::properties RPLidar::get_properties() {

    // create properties return message
    struct sdk::Camera::properties prop;
    prop.supports_pcd = true;

    std::cout << "returning properties" << std::endl;
    return prop;
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

// GetGeometries
std::vector<sdk::GeometryConfig> RPLidar::get_geometries(const sdk::AttributeMap& extra) {
    std::cerr << "get_geometries not implemented" << std::endl;
    return std::vector<sdk::GeometryConfig>{};
}

// DoCommand
sdk::AttributeMap RPLidar::do_command(sdk::AttributeMap command) {
    std::cerr << "do_command not implemented" << std::endl;
    return sdk::AttributeMap{};
}

}  // namespace rplidar
}  // namespace viam