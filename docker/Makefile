build:
	docker build --build-arg FOOBAR="TerraBot" -t terrabot_image .

# Need to be able to connect to already running container (exec)

run:
	docker run -it --name terrabot_container --volume .:/home/robotanist/User -p 5901:5901 terrabot_image

stop:
	docker container stop terrabot_container
	docker container rm terrabot_container

run_once:
	docker run -it --rm --name terrabot_container --volume .:/home/robotanist/User -p 5901:5901 terrabot_image

save:
	docker save -o terrabot_image.tar terrabot_image

load:
	docker load -i terrabot_image.tar
