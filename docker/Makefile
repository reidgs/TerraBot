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

run: docker_build
	docker run -it --rm --name robotanist_container --volume "$(shell pwd)":/home/robotanist -p 8000:8000

save:
	docker terrabot_2025 save -o terrabot_2025.tar

load:
	docker load -i terrabot_2025.tar

stop:
	docker container stop robotanist_container
	docker container rm robotanist_container

build:
	docker build --build-arg HOST_UID=$(shell id -u) -t terrabot_2025 -f ./terrabot.docker .

# Sets the build type to Debug.
set_debug:
	$(eval build_type=Debug)

# Ensures that the build type is debug before running all target.
debug_all: | set_debug all

clean:
	rm -rf build bin lib
