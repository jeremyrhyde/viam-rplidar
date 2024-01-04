cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-rplidar
    DESCRIPTION "Viam Module for the RPLiDAR"
    HOMEPAGE_URL https://github.com/jeremyrhyde/viam-rplidar
    LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 17)

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
# find_package(realsense2 REQUIRED)
# find_package(viam-cpp-sdk REQUIRED)

# The image encoders
# set(THIRD_PARTY_SOURCES src/third_party/fpng.cpp src/third_party/lodepng.cpp)

# construct the library
# add_library(viamrealsense src/camera_realsense.cpp ${THIRD_PARTY_SOURCES})
add_library(viamrplidar src/rplidar.cpp)# ${THIRD_PARTY_SOURCES})

target_link_libraries(viamrplidar 
    PUBLIC
        viam-cpp-sdk::viamsdk
        #realsense2::realsense2
	${CMAKE_DL_LIBS}
)

# Add all the flags
target_compile_features(viamrplidar PUBLIC
    cxx_std_17
)
target_compile_options(viamrplidar PRIVATE 
    -O2 
    -Wredundant-move 
    -Wpessimizing-move 
)

# construct the module executable
add_executable(viam-rplidar main.cpp)

target_link_libraries(viam-rplidar PRIVATE viamrplidar)

target_compile_options(viam-rplidar PRIVATE 
    -O2 
    -Wredundant-move 
    -Wpessimizing-move 
)

enable_testing()
# add_subdirectory(test)