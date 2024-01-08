cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-rplidar
    DESCRIPTION "Viam Module for the RPLiDAR"
    HOMEPAGE_URL https://github.com/jeremyrhyde/viam-rplidar
    LANGUAGES CXX
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include(FetchContent)
FetchContent_Declare(
  viam-cpp-sdk
  GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk.git
  GIT_TAG 2e5fd316ce43410fb59778220fb53128d613892a
  CMAKE_ARGS -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON
  EXCLUDE_FROM_ALL
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(viam-cpp-sdk)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

file(GLOB SOURCES "./src/third_party/rplidar_sdk/sdk/src/rplidar_driver.cpp")

# construct the library
add_library(viamrplidar ${SOURCES} src/rplidar.cpp)
target_link_libraries(viamrplidar 
    PUBLIC
        viam-cpp-sdk::viamsdk
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src/third_party/rplidar_sdk/output/Darwin/Release/libsl_lidar_sdk.a
	${CMAKE_DL_LIBS}
)

target_compile_definitions(viamrplidar PUBLIC _MACOS)

# Add all the flags
target_compile_features(viamrplidar PUBLIC
    cxx_std_17
)

target_include_directories(viamrplidar PUBLIC src/third_party/rplidar_sdk/sdk/include src/third_party/rplidar_sdk/sdk/src)

target_compile_options(viamrplidar PRIVATE 
    -O2 
)

# construct the module executable
set(CMAKE_CXX_STANDARD 17)

add_executable(viam-rplidar main.cpp)

target_link_libraries(viam-rplidar PRIVATE viamrplidar)

target_compile_options(viam-rplidar PRIVATE 
    -O2 
)

enable_testing()