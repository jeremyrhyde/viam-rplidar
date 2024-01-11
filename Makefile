BUILD_CHANNEL?=local
BUILD_DIR = build/$(shell uname -s)-$(shell uname -m)
SHELL := /usr/bin/env bash 

ifneq (, $(shell which brew))
	EXTRA_CMAKE_FLAGS := -DCMAKE_PREFIX_PATH=$(shell brew --prefix) -DQt5_DIR=$(shell brew --prefix qt5)/lib/cmake/Qt5
	export PKG_CONFIG_PATH := $(shell brew --prefix openssl@3)/lib/pkgconfig
endif
# format the source code
format: src/*.cpp src/*.hpp test/*.cpp
	ls src/*.cpp src/*.hpp test/*.cpp | xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}"

build-module:
	cmake -B $(BUILD_DIR) -G Ninja ${EXTRA_CMAKE_FLAGS} && \
	cmake --build $(BUILD_DIR) -j 2
	cp $(BUILD_DIR)/viam-rplidar viam-rplidar

default: build-module

all: setup build-module

clean:
	rm -rf build

clean-sdk:
	rm -rf src/third_party/rplidar_sdk/obj
	rm -rf src/third_party/rplidar_sdk/output

clean-all: clean clean-sdk
	git clean -fxd

install:
	sudo cp ${BUILD_DIR}/viam-rplidar /usr/local/bin/viam-rplidar
	sudo chmod +x /usr/local/bin/viam-rplidar

setup: install-dependencies submodule-initialized
	cd src/third_party/rplidar_sdk && $(MAKE) all

install-dependencies:
ifneq (, $(shell which brew))
	brew update
	brew install boost wget git abseil ninja pkg-config protobuf
else ifneq (, $(shell which apt-get))
	$(warning  "Installing cartographer external dependencies via APT.")
	$(warning "Packages may be too old to work with this project.")
	sudo apt-get update
	sudo apt-get install -y cmake build-essential libabsl-dev libboost-all-dev libgrpc++-dev libprotobuf-dev pkg-config ninja-build \
		protobuf-compiler-grpc git wget
else
	$(error "Unsupported system. Only apt and brew currently supported.")
endif

submodule-initialized:
	@if [ ! -d "src/third_party/rplidar_sdk/sdk" ]; then \
		echo "Submodule was not found. Initializing..."; \
		git submodule update --init --recursive; \
	else \
		echo "Submodule found successfully"; \
	fi

module.tar.gz: clean-sdk setup build-module
	tar czf $@ *.sh viam-rplidar