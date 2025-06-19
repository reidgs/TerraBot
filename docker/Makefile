USER_DIR ?= "."

build:
	docker build --build-arg FOOBAR="TerraBot" -t terrabot_image .

rebuild:
	docker build --no-cache --build-arg FOOBAR="TerraBot" -t terrabot_image .

# Need to be able to connect to already running container (exec)

run:
	docker run -it ${ONCE} --name terrabot_container --volume \
	           ${USER_DIR}:/home/robotanist/User -p 5901:5901 terrabot_image

run_once:
	make run ONCE="--rm"

exec:
	docker exec -it terrabot_container bash

stop:
	docker container stop terrabot_container
	docker container rm terrabot_container
