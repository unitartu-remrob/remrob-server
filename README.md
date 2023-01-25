# REMROB: a web-based robotics learning and development environment

|   |   |
|---|---|
![GNOME ROS VNC](./docs/user-panel.png) | ![GNOME ROS VNC](./docs/user-session.png)
![GNOME ROS VNC](./docs/sim-panel.png) | ![GNOME ROS VNC](./docs/browser-desktop.png)

## !! Clone recursively !!
`git clone --recursive https://github.com/unitartu-remrob/remrob-server`

# Docker installation
The [ROS-VNC image](https://github.com/unitartu-remrob/remrob-docker) with user's development environment must be built beforehand (7 GB, will take some time):

	bash build-image.sh

The application can be orchestrated with docker-compose. Specify the application's timezone in the docker-compose file and launch!

	docker-compose up --build

The dockerized application will connect ports 80, 5000, 9000 and 6085 on the host, make sure they are available.

Access on http://127.0.0.1:80


Default user email: **admin**

Default user password: **admin**

Only simulation environments are available with the docker installation, to support physical robots proceed with the manual installation.

### Hardware acceleration

To enable hardware accelerated containers with nvidia video cards:
1. Install the [nvidia-docker-runtime](https://docs.nvidia.com/ai-enterprise/deployment-guide-vmware/0.1.0/docker.html)
2. In the j2 template at `server/compose/templates/local.j2`:

a) VGL uses the host's display for 3D rendering, default display nr. assumed is **:0**, can be changed to a different one in the following X11 server socket mount (**X1** in the example):

```
volumes: 
      - /tmp/.X11-unix/X1:/tmp/.X11-unix/X0:ro
#	               ^
```

b) To enable GPU resources uncomment:
```
deploy:
resources:
	reservations:
	devices:
		- driver: nvidia
		capabilities: [gpu, utility, graphics]
```
3. Rebuild the compose templates

```
docker-compose build --no-cache node-container-api
```
Alternatively make the same edits within the node container and regenerate with:

	docker exec -w /remrob-server/compose remrob-server_node-container-api_1 python3 compose_generator.py

&nbsp;

# Manual installation 

## Requirements

This software is known to work with the following:

- Ubuntu 20.04
- [Docker Engine 20.10](https://docs.docker.com/engine/install/ubuntu/)
- [docker-compose v2.12.2](https://docs.docker.com/compose/install/other/)
- nginx v1.18.0
- PostgreSQL 12.12
- Nodejs v16.13.0
- NPM v8.19.2
- Python 3.8
- pip 20.0.2

Additional requirements:

- [websockify](https://github.com/novnc/websockify) (available via apt: `sudo apt install websockify`)
- Jinja2 template engine | `pip install Jinja2`
- [nvidia-docker-runtime](https://docs.nvidia.com/ai-enterprise/deployment-guide-vmware/0.1.0/docker.html) (follow the deployment guide)

### 1) Build the VNC/ROS docker image 
```
git clone --recursive https://github.com/unitartu-remrob/remrob-docker
docker build -t robotont:base ./remrob-docker
```
### 2) Build and run the booking backend
```
# Build frontend
cd remrob-webapp
npm install && npm run build

# Install py modules
pip install -r requirements.txt

# Initialise the database
python3 -m flask db init
python3 -m flask db migrate
python3 -m flask db upgrade

# Run the flask backend
npm start
```
### 3) Build and run the container API
```
cd server && npm install
# This will start both websockify and node servers:
npm run prod
```
### 4) Generate compose templates for starting ROS-VNC containers

*Optional*: change network config at `server/compose/config` & `server/websockify-token.cfg` for physical robot support
```
cd remrob-server/server/compose
python3 compose-generator.py
```
### 5) Set up nginx
```
# Copy configuration and restart
sudo cp remrob-server/nginx.conf /etc/nginx/sites-enabled/default
sudo systemctl restart nginx
```

Access via http://localhost

# Network specification for enabling containers to connect to physical robots

To allow full communication between the containers and the robotonts *macvlan* docker network type can be used. It requires an ethernet interface with promiscuous mode enabled (does not work with a wireless interface).


### Create the macvlan docker network
1. Toggle promiscuous mode

`sudo ifconfig {eth_iface} promisc`

2. Create the docker network

`docker network create -d macvlan -o parent={eth_iface} --gateway={router gateway} --subnet={router subnet} --ip-range={available range of ip's on the router} remrob`

**Example:**
`docker network create -d macvlan -o parent=enp46s0 --gateway=192.168.0.1 --subnet=192.168.0.0/24 --ip-range=192.168.0.192/27 remrob`

### Make the containers available to host
1. Make a custom macvlan interface

`ip link add {if_name} link {your_eth_if} type macvlan mode bridge`

**Example:**
`ip link add my_nic link enp46s0 type macvlan mode bridge`

2. Give it an IP address

`ip addr add {ip_addr} dev {if_name}`

**Example:**
`ip addr add 192.168.0.224/32 dev my_nic`

3. Enable it

`ip link set {if_name} up`

4. Route all the containers through it

`ip route add {docker_macvlan_ip_range} dev {if_name}`

**Example:**
`ip route add 192.168.0.192/27 dev my_nic`

Based on [this article](https://blog.oddbit.com/post/2018-03-12-using-docker-macvlan-networks/)

### Limitations & issues:

- After starting the container it initially takes some time before application windows get drawn.
- With more than 9 containers active the vnc servers running in new containers become unresponsive (10+), possibly something to do with the shared cgroup
- Cannot edit files that require sudo privileges with GUI applications (e.g. `sudo gedit /etc/hosts`), must use a CLI editor (e.g. nano)
- SYS_ADMIN container privileges currently required to run systemd, which is needed for gnome


&nbsp;

&nbsp;

# Acknowledgments

Completed with the support by IT Academy Programme of Education and Youth Board of Estonia.

Valminud Haridus- ja Noorteameti IT Akadeemia programmi toel.
