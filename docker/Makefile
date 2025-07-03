USER_DIR ?= "."

build:
	docker build ${REBUILD} --build-arg FOOBAR="TerraBot" -t terrabot_image_ros2 .

rebuild:
	make build REBUILD="--no-cache"

# Need to be able to connect to already running container (exec)

run:
	docker run -it ${ONCE} --name terrabot_container_ros2 --volume \
	           ${USER_DIR}:/home/robotanist/User -p 5901:5901 terrabot_image_ros2

run_once:
	make run ONCE="--rm"

exec:
	docker exec -it terrabot_container_ros2 bash

stop:
	docker container stop terrabot_container_ros2
	docker container rm terrabot_container_ros2

run_once:
	docker run -it --rm --name terrabot_container_ros2 --volume .:/home/robotanist/User -p 5902:5901 terrabot_image_ros2

