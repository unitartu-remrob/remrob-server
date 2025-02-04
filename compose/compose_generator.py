import yaml
from jinja2 import Environment, FileSystemLoader

if __name__ == "__main__":

	robo_config_macvlan = yaml.safe_load(open('config/config-macvlan.yaml'))
	robo_config_local = yaml.safe_load(open('config/config-local.yaml'))

	# Load templates file from templtes folder
	env = Environment(loader = FileSystemLoader('./templates'), trim_blocks=True, lstrip_blocks=True)

	template_macvlan = env.get_template('macvlan.j2')
	template_local = env.get_template('local.j2')

	for i, robot in enumerate(robo_config_macvlan):
		
		output = template_macvlan.render(
			name=robot["name"],
			robotont=robot["master-name"],
			master_ip=robot["master-ip"],
			self_ip=robot["self-ip"]
		)
		file = open(f"./macvlan/robo-{i+1}.yaml", "w")
		file.write(output)
		file.close()
	
	for i, robot in enumerate(robo_config_local):
		
		output = template_local.render(
			name=robot["name"],
			port=robot["port"]
		)
		file = open(f"./local/robosim-{i+1}.yaml", "w")
		file.write(output)
		file.close()

