## API for starting and managing live noVNC container sessions

Requires websockify, can install with apt (`sudo apt install websockify`)

1. Install the server modules

    `npm install`

2. Run the noVNC client on port 6085

    `npm run vnc-client`

    If the 6085 port is being proxied via `/novnc` path then the containers running VNC servers can now be excessed with a link of the following format:

    http://localhost/novnc/vnc.html?autoconnect=true&resize=remote&password=remrob&path=novnc?token=robo-1

3. Run the server

    `npm run server`


### Dev server with hot updates

`npm run dev`

---

To change to ip's of different subnet being used (other than 192.168.200.192/27), modify the compose config under `compose/config` and run the Jinja2 template parser (`compose_generator.py`), to enable connections also change `websockify-token.cfg`


### Running as an auto-restarting background process under systemd

Install the pm2 daemon process manager (available via npm)

    npm install pm2@latest -g

    pm2 startup // follow instructions

    pm2 start ecosystem.config.js

Restart with changes

    pm2 reload remrob

Stream logs

    pm2 logs
