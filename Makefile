# format the source code
format: src/*.cpp src/*.hpp test/*.cpp
	ls src/*.cpp src/*.hpp test/*.cpp | xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}"

viam-rplidar: src/*
	rm -rf build/ && \
	mkdir build && \
	cd build && \
	cmake -G Ninja .. && \
	ninja all -j 4 && \
	cp viam-rplidar ../

ensure-submodule-initialized:
	@if [ ! -d "src/thrid_party/rplidar_sdk/rplidar_sdk" ]; then \
		echo "Submodule was not found. Initializing..."; \
		git submodule update --init; \
	else \
		echo "Submodule found successfully"; \
	fi

default: viam-rplidar

all: default

clean:
	rm -rf build
	rm -rf viam-rplidar

clean-all: clean
	git clean -fxd

setup: install-dependencies ensure-submodule-initialized

install-dependencies:
	# setup step for cloud builds
	sudo apt-get update
	sudo ./apt-setup.sh
	sudo apt-get install -qy cmake