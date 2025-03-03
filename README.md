# REMROB: a web-based robotics learning and development environment

|   |   |
|---|---|
![GNOME ROS VNC](./docs/user-panel.png) | ![GNOME ROS VNC](./docs/user-session.png)
![GNOME ROS VNC](./docs/sim-panel.png) | ![GNOME ROS VNC](./docs/browser-desktop.png)

## Introduction

Remrob is a web application for a remote web lab that offers an authentic ROS development experience by serving in-browser desktop workstations with the help of [noVNC](https://github.com/novnc/noVNC). Docker containers are used to encapsulate the workstations of which there are two types - simulation and physical robot enabled environments. The users are able to reserve access to the remote lab through a time slot booking module.

See ["Open Remote Web Lab for Learning Robotics and ROS With Physical and Simulated Robots in an Authentic Developer Environment"](https://ieeexplore.ieee.org/document/10480223) published in IEEE Transactions on Learning Technologies for more details.

Demo video: https://www.youtube.com/watch?v=FGVpwIwRrwc

## Remrob installation

See https://github.com/unitartu-remrob/remrob-setup for installation instructions.

---

# remrob-server: ROS-VNC container management API

The remrob-server is a Node.js app for launching [ROS-VNC containers](https://github.com/unitartu-remrob/remrob-docker) via an API.

The app provides authentication and authorization layers, and uses Docker compose to craft user-specific container environments.

## Requirements

- Node v20
- Docker & Docker compose

## Setup

```
npm install
```

### Dev server with hot updates

```
npm run dev
```

### Production server

```
npm run server
```

### Running multiple instances with pm2

Install the pm2 daemon process manager (available via npm)

    npm install pm2@latest -g

Start with pm2:

    npm run pm2:cluster

Run as a persistent background process:

    pm2 startup // follow instructions

    pm2 start ecosystem.config.cjs

Check status:

    pm2 status remrob

Restart with changes:

    pm2 reload remrob

Stream logs:

    pm2 logs


&nbsp;&nbsp;

# Acknowledgments

Completed with the support by IT Academy Programme of Education and Youth Board of Estonia.

Valminud Haridus- ja Noorteameti IT Akadeemia programmi toel.