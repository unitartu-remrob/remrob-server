Clone recursively, project uses [existing robotont software](https://github.com/robotont)

A vnc-ros-gnome image (inspired by and built upon from this [image](https://github.com/darkdragon-001/Dockerfile-Ubuntu-Gnome))

## Requirements

- Docker & docker-compose(1.28.0+)
- [nvidia-docker-runtime](https://docs.docker.com/config/containers/resource_constraints/#gpu) (comes with nvidia-docker2)


### Building the image

1. `docker build -t robotont:base .`

### Running the container

1. Change the `extra-hosts` directive to your robot's hostname and IP address (e.g. "robotont-7:192.168.0.39")

2. `docker-compose up`

3. All the ports of the container are shared with the host in the "host" network mode. Connect to port 5901 of your machine via any VNC client to connect to the container