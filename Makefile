SHELL := /bin/bash
HOST_UID := $(shell id -u)

CONTAINER_NAME:=pi-camera-ros-driver

# make run
# Run this command to build the container
build:
	docker build -t $(CONTAINER_NAME) --build-arg HOST_UID=$(HOST_UID) .

# make run
# Run this command to just run the container in the background
run: build
	docker compose up -d

# make view
# run this command to build and run the container and view the output
view: run
	docker logs $(shell docker ps --filter "ancestor=$(CONTAINER_NAME)" --format "{{.ID}}" | head -n 1) -f


# make exec
# run this command the build and run the container and then runs a shell in the container too
exec: run
	docker exec -it $(shell docker ps --filter "ancestor=$(CONTAINER_NAME)" --format "{{.ID}}" | head -n 1) bash

# make stop
#run this command to stop the container in the background
stop:
	docker compose down