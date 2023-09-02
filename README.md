# REMROB: a web-based robotics learning and development environment

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

## Installation 
### 1) Build the VNC/ROS docker image 
```
git clone --recursive https://github.com/unitartu-remrob/remrob-docker
cd remrob-docker
docker build -t robotont:base .
```
### 2) Build frontend
```
git clone https://github.com/unitartu-remrob/remrob-webapp
cd remrob-webapp
npm install && npm run build
```
### 3) Build and run the container API
```
git clone --recursive https://github.com/unitartu-remrob/remrob-server
cd remrob-server/server
npm install
# This will start both websockify and node servers:
npm run prod
```
### 4) Generate docker-compose templates

*Optional*: change network config at `remrob-server/server/compose/config` for physical robot support
```
cd remrob-server/server/compose
python3 compose-generator.py
```
### 5) Build and run the booking backend
```
cd remrob-webapp
pip install -r requirements.txt
npm run dev-server
```
### 6) Set up nginx
```
deb https://nginx.org/packages/ubuntu/ focal nginx
sudo apt update && sudo apt install nginx
sudo cp nginx.conf /etc/nginx/sites-enabled/default
sudo systemctl start nginx
```

Access via http://localhost

# Network specification for enabling containers to connect to physical robots

To allow full communication between the containers and the robotonts *macvlan* docker network type can be used. It requires an ethernet interface with promiscuous mode enabled (does not work with a wireless interface).


### Create the macvlan docker network
1. Toggle promiscuous mode

`sudo ifconfig {eth_iface} promisc`

2. Create the docker network

`docker network create -d macvlan -o parent={eth_iface} --gateway={router gateway} --subnet={router subnet} --ip-range={available range of ip's on the router} {network_name}`

**Example:**
`docker network create -d macvlan -o parent=enp46s0 --gateway=192.168.0.1 --subnet=192.168.0.0/24 --ip-range=192.168.0.192/27 pub_net`

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

- Cannot edit files that require sudo privileges with GUI applications (e.g. `sudo gedit /etc/hosts`), must use a CLI editor (e.g. nano)
- SYS_ADMIN container privileges currently required to run systemd, which is needed for gnome


&nbsp;

&nbsp;

# Acknowledgments

Completed with the support by IT Acadamy Programme of Education and Youth Board of Estonia.

Valminud Haridus- ja Noorteameti IT Akadeemia programmi toel.
