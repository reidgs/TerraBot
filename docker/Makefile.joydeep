# Clang is a good compiler to use during development due to its faster compile
# times and more readable output.
# C_compiler=/usr/bin/clang
# CXX_compiler=/usr/bin/clang++

# GCC is better for release mode due to the speed of its output, and its support
# for OpenMP.
C_compiler=/usr/bin/gcc
CXX_compiler=/usr/bin/g++

#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

.SILENT:
all: build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

docker_all: docker_build_q
	docker run --rm --volume "$(shell pwd)":/home/dev/cs378_starter cs378_starter "cd cs378_starter && make -j"

docker_shell: docker_build_q
	if [ $(shell docker ps -a -f name=cs378_starter_shell | wc -l) -ne 2 ]; then docker run -dit --name cs378_starter_shell --volume "$(shell pwd)":/home/dev/cs378_starter --workdir /home/dev/cs378_starter -p 10272:10272 cs378_starter; fi
	docker exec -it cs378_starter_shell bash -l

docker_stop:
	docker container stop cs378_starter_shell
	docker container rm cs378_starter_shell

docker_build:
	docker build --build-arg HOST_UID=$(shell id -u) -t cs378_starter .

docker_build_q:
	docker build -q --build-arg HOST_UID=$(shell id -u) -t cs378_starter .

# Sets the build type to Debug.
set_debug:
	$(eval build_type=Debug)

# Ensures that the build type is debug before running all target.
debug_all: | set_debug all

clean:
	rm -rf build bin lib

build/CMakeLists.txt.copy: CMakeLists.txt Makefile
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) \
		-DCMAKE_CXX_COMPILER=$(CXX_compiler) \
		-DCMAKE_C_COMPILER=$(C_compiler) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy
