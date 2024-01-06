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
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>


namespace viam {
namespace rplidar {

namespace rpsdk = rp::standalone::rplidar;

std::tuple<RPLidarProperties, bool, bool> RPLidar::initialize(sdk::ResourceConfig cfg) {
    return {};
}

// ----------------------------------------- SETUP HELPERS ----------------------------------------

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

float get_angle(const rplidar_response_measurement_node_hq_t &node) {
  return node.angle_z_q14 * 90.f / (1 << 14);
}

float get_distance(const rplidar_response_measurement_node_hq_t &node) {
  return node.dist_mm_q2 / 4.0f / 1000; 
}

float get_quality(const rplidar_response_measurement_node_hq_t &node) {
  return node.quality >> 2;
}

pcl::PointXYZI create_point(float dist, float angle, float intensity) {
    pcl::PointXYZI point;
    point.x = dist*cos(angle * M_PI / 180);
    point.y = dist*sin(angle * M_PI / 180);
    point.z = 0.0;
    point.intensity = intensity;

    return point;
}

std::vector<unsigned char> RPLidar::scan(int num_scans) {
    // Get scan nodes
    size_t count = default_node_size * num_scans;
    rplidar_response_measurement_node_hq_t nodes[count * sizeof(rplidar_response_measurement_node_hq_t)];

    if (driver->grabScanDataHq(nodes, count) != RESULT_OK) {
        std::cerr << "grabScanDataHq failed to grab data" << std::endl;
        return {};
    }

    if (driver->ascendScanData(nodes, count) != RESULT_OK) {
        std::cerr << "ascendScanData failed to sort data" << std::endl;
        return {};
    }

    // Create pointcloud
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width    = 1;
    cloud.height   = count;
    cloud.is_dense = true;
    cloud.resize(cloud.width * cloud.height);

    // Loop over pointcloud
    for (int i = 0; i < count; ++i) {
        float dist_value = get_distance(nodes[i]);
        if (dist_value < min_range_mm) {
            continue;
        }
        float angle_value = get_angle(nodes[i]);
        float intensity_value = get_quality(nodes[i]);

        cloud.push_back(create_point(dist_value, angle_value, intensity_value));
    }

    std::vector<unsigned char> bytes = {'a', 'b', 'c'};

    return bytes;
}

// -------------------------------------------- SETUP -------------------------------------------

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
        throw std::runtime_error("could not start RPLidat");
    } else {
        std::cout << "started RPLidar... " << std::endl;
    }

    // Start background polling process
    if (!start_polling()) {
        rpsdk::RPlidarDriver::DisposeDriver(driver);
        throw std::runtime_error("could not start background polling process");
    } else {
        std::cout << "started polling process... " << std::endl;
    }
}

RPLidar::~RPLidar() {
    if (!stop_motor(driver)) {
        std::cerr << "issue occured stopping the motor" << std::endl;
    }
    rpsdk::RPlidarDriver::DisposeDriver(driver);
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

bool RPLidar::start() {
    if (!start_motor(driver)) {
         std::cerr << "issue occurred starting motor" << std::endl;
         return false;
    }

    // rp.nodes = gen.New_measurementNodeHqArray(defaultNodeSize)
    // Perform warm up sequence
    return true;
}

bool RPLidar::start_polling() {
    return true;
}

// ---------------------------------------- CAMERA METHODS ---------------------------------------

// Reconfigure
void RPLidar::reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) {
    std::cerr << "reconfigure not implemented" << std::endl;
    return;
}

// GetPointCloud
sdk::Camera::point_cloud RPLidar::get_point_cloud(std::string mime_type, const sdk::AttributeMap& extra) {

    std::vector<unsigned char> pcd_bytes = scan(1);
    if (sizeof(pcd_bytes)) {
        throw std::runtime_error("error no point_cloud data returned from scan");
    }

    // create point_cloud return message
    sdk::Camera::point_cloud point_cloud;
    point_cloud.mime_type = "pointcloud/pcd";
    point_cloud.pc = pcd_bytes;

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