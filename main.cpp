#include <viam/sdk/components/camera/camera.hpp>
#include <viam/sdk/module/service.hpp>

#include "src/rplidar.hpp"

using namespace viam::sdk;

int main(int argc, char* argv[]) {
    std::cout << "hi im paul" << std::endl;
    std::shared_ptr<ModelRegistration> mr = std::make_shared<ModelRegistration>(
        Camera::static_api(), Model{
            viam::rplidar::kAPINamespace, 
            viam::rplidar::kAPIType, 
            viam::rplidar::kAPISubtype,
        },
        [](Dependencies deps, ResourceConfig cfg) { return std::make_shared<viam::rplidar::RPLidar>(deps, cfg); }
    );

    auto service = std::make_shared<ModuleService>(argc, argv, std::vector<std::shared_ptr<ModelRegistration>>{mr});

    service->serve();

    return 0;
}
