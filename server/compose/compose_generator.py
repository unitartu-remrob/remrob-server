import yaml
import os
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

    for i, robot in enumerate(robo_config_local):
        fps_out_file = f"local/temp/robosim-{i+1}/FPS_out.txt"
        gazebo_fps_out_file = f"local/temp/robosim-{i+1}/GAZEBO_FPS_out.txt"

        if not os.path.exists(fps_out_file):
            os.mknod(fps_out_file)

        if not os.path.exists(gazebo_fps_out_file):
            os.mknod(gazebo_fps_out_file)
        # rviz_fps_log = open(f"config/temp/robosim-{i+1}/FPS_out.txt", 'w')
        # rviz_fps_log.close()
        # gazebo_fps_log = open(f"config/temp/robosim-{i+1}/GAZEBO_FPS_out.txt", 'w')
        # gazebo_fps_log.close()

