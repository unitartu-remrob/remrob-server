#!/usr/bin/python3

import yaml
from jinja2 import Environment, FileSystemLoader
import argparse

parser = argparse.ArgumentParser(description='Compose Generator')
parser.add_argument('--nvidia', action='store_true', help='use HW accelerated container compose templates')
parser.add_argument('--xsocket', type=str, help='X11 display socket to use for VirtualGL')
parser.add_argument('--macvlan', type=str, help='Docker MACVLAN network name')
parser.add_argument('--sim-net', type=str, help='Docker bridge network name (for simulation containers)')

args = parser.parse_args()

USE_NVIDIA_RUNTIME = args.nvidia
DISPLAY_SOCKET = "X1" if args.xsocket == None else args.xsocket
MACVLAN_NETWORK_NAME = "remrob" if args.macvlan == None else args.macvlan
SIMULATION_NETWORK_NAME = "sim_net" if args.sim_net == None else args.sim_net

# Default params
#----------------
if USE_NVIDIA_RUNTIME:
	DEFAULT_IMAGE_NOETIC = "remrob:noetic-cudagl"
	DEFAULT_IMAGE_JAZZY = "remrob:jazzy-cudagl"
else:
	DEFAULT_IMAGE_NOETIC = "remrob:noetic-base"
	DEFAULT_IMAGE_JAZZY = "remrob:jazzy-base"
#----------------

# Constants
ROS_VERSION_NOETIC = "noetic"
ROS_VERSION_JAZZY = "jazzy"
MACVLAN = 'macvlan'
LOCAL = 'local'
NVIDIA_SUFFIX = '-nvidia'


TEMPLATES = Environment(loader = FileSystemLoader('./templates'), trim_blocks=True, lstrip_blocks=True)

CONFIG_MACVLAN = yaml.safe_load(open('config/config-macvlan.yaml'))
CONFIG_LOCAL = yaml.safe_load(open('config/config-local.yaml'))


def get_template(ros_version, template_type):
	if (USE_NVIDIA_RUNTIME):
		file = f'{ros_version}/{template_type}{NVIDIA_SUFFIX}.j2'
	else:
		file = f'{ros_version}/{template_type}.j2'

	return TEMPLATES.get_template(file)


def write_generated_file(file_name, output):
	file = open(file_name, "w")
	file.write(output)
	file.close()


def render_macvlan_templates(ros_version):
	j2_template = get_template(ros_version, MACVLAN)

	default_macvlan_params = {
		"display_socket": DISPLAY_SOCKET,
		"network_name": MACVLAN_NETWORK_NAME
	}

	for i, robot in enumerate(CONFIG_MACVLAN):
		output = ""

		if ros_version == ROS_VERSION_NOETIC:
			output = j2_template.render(
				name=robot["name"],
				robot_hostname=robot["master_name"],
				master_ip=robot["master_ip"],
				self_ip=robot["self_ip"],
				image=DEFAULT_IMAGE_NOETIC,
				**default_macvlan_params
			)
		elif ros_version == ROS_VERSION_JAZZY:
			output = j2_template.render(
				name=robot["name"],
				self_ip=robot["self_ip"],
				ros_domain_id=robot["ros_domain_id"],
				image=DEFAULT_IMAGE_JAZZY,
				**default_macvlan_params
			)
		
		write_generated_file(f"./{MACVLAN}/{ros_version}/robo-{i+1}.yaml", output)


def render_local_templates(ros_version):
	j2_template = get_template(ros_version, LOCAL)

	default_sim_params = {
		"display_socket": DISPLAY_SOCKET,
		"network_name": SIMULATION_NETWORK_NAME
	}

	image = DEFAULT_IMAGE_NOETIC if ros_version == ROS_VERSION_NOETIC else DEFAULT_IMAGE_JAZZY
	
	for i, robot in enumerate(CONFIG_LOCAL):
		output = j2_template.render(
			name=robot["name"],
			port=robot["port"],
			image=image,
			ros_domain_id=50+i+1 if ros_version == ROS_VERSION_JAZZY else None,
			**default_sim_params
		)
		write_generated_file(f"./{LOCAL}/{ros_version}/robosim-{i+1}.yaml", output)
			

if __name__ == "__main__":
	render_macvlan_templates(ROS_VERSION_NOETIC)
	render_local_templates(ROS_VERSION_NOETIC)

	render_macvlan_templates(ROS_VERSION_JAZZY)
	render_local_templates(ROS_VERSION_JAZZY)