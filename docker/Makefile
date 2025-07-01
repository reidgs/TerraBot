build:
	docker build -f Dockerfile.ros2 --build-arg FOOBAR="TerraBot" -t terrabot_image_ros2 .

rebuild:
	docker build -f Dockerfile.ros2 --no-cache --build-arg FOOBAR="TerraBot" -t terrabot_image_ros2 .

# Need to be able to connect to already running container (exec)

run:
	docker run -it --name terrabot_container_ros2 --volume .:/home/robotanist/User -p 5901:5901 terrabot_image_ros2

stop:
	docker container stop terrabot_container_ros2
	docker container rm terrabot_container_ros2

run_once:
	docker run -it --rm --name terrabot_container_ros2 --volume .:/home/robotanist/User -p 5902:5901 terrabot_image_ros2

save:
	docker save -o terrabot_image_ros2.tar terrabot_image_ros2

load:
	docker load -i terrabot_image_ros2.tar
