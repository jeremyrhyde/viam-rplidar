# format the source code
format: src/*.cpp src/*.hpp test/*.cpp
	ls src/*.cpp src/*.hpp test/*.cpp | xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}"

build: src/*
	rm -rf build/ && \
	mkdir build && \
	cd build && \
	cmake -G Ninja .. && \
	ninja all -j 4 && \
	cp viam-rplidar ../

default: build

all: setup build

clean:
	rm -rf build
	rm -rf viam-rplidar

clean-all: clean
	git clean -fxd

setup: install-dependencies ensure-submodule-initialized

install-dependencies:
ifneq (, $(shell which brew))
	brew update
	brew install boost wget git abseil ninja pkg-config protobuf
else ifneq (, $(shell which apt-get))
	$(warning  "Installing cartographer external dependencies via APT.")
	$(warning "Packages may be too old to work with this project.")
	sudo apt-get update
	sudo apt-get install -qqy build-essential libabsl-dev libboost-all-dev libgrpc++-dev libprotobuf-dev pkg-config ninja-build \
		protobuf-compiler-grpc git wget
	sudo apt-get install -qy cmake
else
	$(error "Unsupported system. Only apt and brew currently supported.")
endif

ensure-submodule-initialized:
	@if [ ! -d "src/thrid_party/rplidar_sdk" ]; then \
		echo "Submodule was not found. Initializing..."; \
		git submodule update --init; \
	else \
		echo "Submodule found successfully"; \
	fi