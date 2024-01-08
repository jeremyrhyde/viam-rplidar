#include "rplidar.hpp"

#include <pthread.h>
#include <signal.h>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>
#include <cmath>
#include <sstream>
#include <iostream>
#include <fstream>

#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>


namespace viam {
namespace rplidar {

namespace rpsdk = rp::standalone::rplidar;

void RPLidar::initialize(sdk::ResourceConfig cfg) {
    return;
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

// Node interface functions : (angle, dist)
float get_angle(const rplidar_response_measurement_node_hq_t &node) {
  return node.angle_z_q14 * 90.f / (1 << 14);
}

float get_distance(const rplidar_response_measurement_node_hq_t &node) {
  return node.dist_mm_q2 / 4.0f / 1000; 
}

// Scan and helper functions to handle pointcloud construction and pcd creation
// : (create_point, point_to_string, PointCloudXYZI_to_pcd_bytes, scan) 
PointXYZI create_point(float dist, float angle) {
    PointXYZI point;
    point.x = dist*cos(angle * M_PI / 180);
    point.y = dist*sin(angle * M_PI / 180);
    point.z = 0.0;
    point.intensity = 16711680;
    return point;
}

std::string point_to_string(PointXYZI point) {
    std::stringstream output;
    output << point.x << " " << point.y << " " << point.z << " " << point.intensity << "\n";
    return output.str();
}

std::vector<unsigned char> PointCloudXYZI_to_pcd_bytes(PointCloudXYZI point_cloud) {

    // Construct PCD header with the number of points in given pointcloud
    std::stringstream header;
    header << "VERSION .7 \n";
    header << "FIELDS x y z rgb \n";
    header << "SIZE 4 4 4 4 \n";
    header << "TYPE F F F I \n";
    header << "COUNT 1 1 1 1 \n";
    header << "WIDTH " << std::to_string(point_cloud.count) << " \n";
    header << "HEIGHT 1 \n";
    header << "VIEWPOINT 0 0 0 1 0 0 0 \n";
    header << "POINTS " << std::to_string(point_cloud.count) << " \n";
    header << "DATA ascii \n";

    // Create PCD with header and points
    std::string pcd = header.str();
    for (int i = 0; i < point_cloud.points.size(); i++) {
        pcd.append(point_to_string(point_cloud.points[i]));
    }

    std::vector<unsigned char> pcd_bytes(pcd.begin(), pcd.end());
    return pcd_bytes;
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
    pc.count = 0;
    for (int i = 0; i < count; ++i) {
        float dist_value = get_distance(nodes[i]);
        if (dist_value < min_range_mm) {
            continue;
        }
        float angle_value = get_angle(nodes[i]);

        pc.points.push_back(create_point(dist_value, angle_value));
        pc.count++;
    }

    return pc;
}

// RPLidar Constructor and Destructor : (validate, RPLidar, ~RPLidar)
std::vector<std::string> validate(sdk::ResourceConfig cfg){
    auto attrs = cfg.attributes();
    if (attrs->count("min_range_mm") == 1) {
        std::shared_ptr<sdk::ProtoType> min_range_mm_proto = attrs->at("min_range_mm");
        auto min_range_mm_value = min_range_mm_proto->proto_value();
        if (min_range_mm_value.has_number_value()) {
            int min_range_mm_num = static_cast<float>(min_range_mm_value.number_value());
            if (min_range_mm_num < 0) {
                throw std::invalid_argument("min_range_mm cannot be negative");
            }
        }
    }   
    return {};
}

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
        serial_port = default_serial_port;
        std::cout << "no serial_path given, attempting to connect using default: " << serial_port << std::endl;
    }

    if (attrs->count("model") == 1) {
        std::shared_ptr<sdk::ProtoType> model_proto = attrs->at("model");
        auto model_value = model_proto->proto_value();
        if (model_value.has_string_value()) {
            std::string model_str = static_cast<std::string>(model_value.string_value());
            rplidar_model = model_str;
            std::cout << "rplidar_model found, using " << rplidar_model << std::endl;
        }
    } else {
        std::cout << "no rplidar_model given" << std::endl;
    }

    if (attrs->count("serial_baudrate") == 1) {
        std::shared_ptr<sdk::ProtoType> serial_baudrate_proto = attrs->at("serial_baudrate");
        auto serial_baudrate_value = serial_baudrate_proto->proto_value();
        if (serial_baudrate_value.has_number_value()) {
            int serial_baudrate_num = static_cast<int>(serial_baudrate_value.number_value());
            serial_baudrate = serial_baudrate_num;
            std::cout << "serial_baudrate_num found, attempting to connect using " << serial_baudrate_num << std::endl;
        }
    } else {
        if (rplidar_model != "") {
            serial_baudrate = baudrate_map[rplidar_model];
            std::cout << "no serial_baudrate given using default for given model: " << serial_baudrate << std::endl;
        } else {
            serial_baudrate = default_baudrate;
            std::cout << "no serial_baudrate or model given, will attempt to connect using default [Baudrate: " << serial_baudrate << "]" << std::endl;
        }
    }

    if (attrs->count("min_range_mm") == 1) {
        std::shared_ptr<sdk::ProtoType> min_range_mm_proto = attrs->at("min_range_mm");
        auto min_range_mm_value = min_range_mm_proto->proto_value();
        if (min_range_mm_value.has_number_value()) {
            float min_range_mm_num = static_cast<float>(min_range_mm_value.number_value());
            min_range_mm = min_range_mm_num;
        }
    } else {
        std::cout << "no min_range_mm given" << std::endl;
    }

    if (attrs->count("use_caching") == 1) {
        std::shared_ptr<sdk::ProtoType> use_caching_proto = attrs->at("use_caching");
        auto use_caching_value = use_caching_proto->proto_value();
        if (use_caching_value.has_bool_value()) {
            float use_caching_bool = static_cast<bool>(use_caching_value.bool_value());
            use_caching = use_caching_bool;
        }
    } 

    std::cout << "use_caching set to " << use_caching << std::endl;

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
        throw std::runtime_error("could not find RPLidar at specified serial_path: " + serial_port + " with [Baudrate: " + std::to_string(serial_baudrate) + "]");
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
    if (use_caching) {
        if (!start_polling()) {
            rpsdk::RPlidarDriver::DisposeDriver(driver);
            throw std::runtime_error("could not start background polling process");
        } else {
            std::cout << "started polling process... " << std::endl;
        }
    }
}

RPLidar::~RPLidar() {
    thread_shutdown = true;
    cameraThread.join();

    if (!stop_motor(driver)) {
        std::cerr << "issue occurred stopping the motor" << std::endl;
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

    //rplidar_model = devinfo.model;

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
        {
            std::unique_lock<std::mutex> lock(cache_mutex);
            cached_pc = pc;
        }
        
        // tell the outside function, the thread is ready and has completed its first loop
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

// ------------------------------------- CAMERA METHODS ----------------------------------------------

// Reconfigure
void RPLidar::reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) {
    std::cerr << "reconfigure not implemented" << std::endl;
    return;
}

// GetPointCloud
sdk::Camera::point_cloud RPLidar::get_point_cloud(std::string mime_type, const sdk::AttributeMap& extra) {
    // Grab point cloud from cache or call scan directly
    PointCloudXYZI pc;
    if (use_caching) {
        {
            std::unique_lock<std::mutex> lock(cache_mutex);
            pc = cached_pc;
        }
    } else {
        pc = scan(driver, min_range_mm);
    }

    // Create point_cloud return message
    sdk::Camera::point_cloud point_cloud;
    point_cloud.mime_type = "pointcloud/pcd";
    point_cloud.pc = PointCloudXYZI_to_pcd_bytes(pc);

    return point_cloud;
}

// GetProperties
sdk::Camera::properties RPLidar::get_properties() {

    // Create properties return message
    struct sdk::Camera::properties prop;
    prop.supports_pcd = true;
 
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