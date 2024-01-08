cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-rplidar
    DESCRIPTION "Viam Module for the RPLiDAR"
    HOMEPAGE_URL https://github.com/jeremyrhyde/viam-rplidar
    LANGUAGES CXX
)
#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include(FetchContent)
FetchContent_Declare(
  viam-cpp-sdk
  GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk.git
  GIT_TAG 2e5fd316ce43410fb59778220fb53128d613892a
  # SOURCE_DIR ${CMAKE_SOURCE_DIR}/../viam-cpp-sdk
  CMAKE_ARGS -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON
  EXCLUDE_FROM_ALL
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(viam-cpp-sdk)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES "./src/thrid_party/rplidar_sdk/sdk/src/rplidar_driver.cpp")

# construct the library
add_library(viamrplidar ${SOURCES} src/rplidar.cpp)
target_link_libraries(viamrplidar 
    PUBLIC
        viam-cpp-sdk::viamsdk
        ${PCL_LIBRARIES}
    PRIVATE
        /Users/jeremyhyde/Development/viam-rplidar/src/thrid_party/rplidar_sdk/output/Darwin/Release/libsl_lidar_sdk.a
	${CMAKE_DL_LIBS}
)

target_compile_definitions(viamrplidar PUBLIC _MACOS)

# Add all the flags
target_compile_features(viamrplidar PUBLIC
    cxx_std_17
)

target_include_directories(viamrplidar PUBLIC src/thrid_party/rplidar_sdk/sdk/include src/thrid_party/rplidar_sdk/sdk/src)

target_compile_options(viamrplidar PRIVATE 
    -O2 
    # -Wredundant-move 
    # -Wpessimizing-move 
)

# construct the module executable
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchkp -pedantic")

add_executable(viam-rplidar main.cpp)

target_link_libraries(viam-rplidar PRIVATE viamrplidar)

target_compile_options(viam-rplidar PRIVATE 
    -O2 
    # -Wredundant-move 
    # -Wpessimizing-move 
)

enable_testing()
# add_subdirectory(test)