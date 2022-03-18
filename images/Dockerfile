FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
        locales \
    && echo "$LANG UTF-8" >> /etc/locale.gen \
    && locale-gen \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -y \
    && apt-get install -y \
        dbus \
        dbus-x11 \
        systemd \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && dpkg-divert --local --rename --add /sbin/udevadm \
    && ln -s /bin/true /sbin/udevadm
RUN systemctl disable systemd-resolved
VOLUME ["/sys/fs/cgroup"]
STOPSIGNAL SIGRTMIN+3
CMD [ "/sbin/init" ]

RUN apt-get update -y \
    && apt-get install -y \
        ca-certificates \
        ubuntu-gnome-desktop \
    && apt-get remove gnome-initial-setup -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install TigerVNC server
# TODO set VNC port in service file > exec command
# TODO check if it works with default config file
# NOTE tigervnc because of XKB extension: https://github.com/i3/i3/issues/1983
RUN apt-get update \
  && apt-get install -y tigervnc-common tigervnc-scraping-server tigervnc-standalone-server tigervnc-viewer tigervnc-xorg-extension \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
# TODO fix PID problem: Type=forking would be best, but system daemon is run as root on startup
#   ERROR tigervnc@:1.service: New main PID 233 does not belong to service, and PID file is not owned by root. Refusing.
#   https://www.freedesktop.org/software/systemd/man/systemd.service.html#Type=
#   https://www.freedesktop.org/software/systemd/man/systemd.unit.html#Specifiers
#   https://wiki.archlinux.org/index.php/TigerVNC#Starting_and_stopping_vncserver_via_systemd
# -> this should be fixed by official systemd file once released: https://github.com/TigerVNC/tigervnc/pull/838
# TODO specify options like geometry as environment variables -> source variables in service via EnvironmentFile=/path/to/env
# NOTE logout will stop tigervnc service -> need to manually start (gdm for graphical login is not working)

COPY tigervnc@.service /etc/systemd/system/tigervnc@.service
ENV DISPLAY=:1
RUN systemctl enable tigervnc@:1

COPY noVNC /usr/share/novnc

RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
        python \
        git \
        pwgen \
        net-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && cp usr/share/novnc/vnc.html /usr/share/novnc/index.html \
    && git clone https://github.com/kanaka/websockify /usr/share/novnc/utils/websockify

COPY novnc.service /etc/systemd/system/novnc.service
RUN systemctl enable novnc


RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
        iputils-ping \
        traceroute \
        vim \
        nano \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
        python3-catkin-tools \
        python3-rosdep \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-tf \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create unprivileged user
# NOTE user hardcoded in tigervnc.service
# NOTE alternative is to use libnss_switch and create user at runtime -> use entrypoint script
ARG UID=1000
ARG USER=kasutaja
RUN useradd ${USER} -u ${UID} -U -d /home/${USER} -m -s /bin/bash
# RUN echo "$USER:password" | chpasswd
RUN apt-get update && apt-get install -y sudo && apt-get clean && rm -rf /var/lib/apt/lists/* && \
    echo "${USER} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${USER}" && \
    chmod 440 "/etc/sudoers.d/${USER}"

ENV USER="${USER}" \
    HOME="/home/${USER}"
USER "${USER}"

WORKDIR "/home/${USER}"

# Set up VNC
RUN mkdir -p $HOME/.vnc
COPY xstartup $HOME/.vnc/xstartup
# RUN echo "password" | vncpasswd -f >> $HOME/.vnc/passwd && chmod 600 $HOME/.vnc/passwd

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

RUN mkdir -p $HOME/catkin_ws/src
WORKDIR $HOME/catkin_ws/src
COPY --chown=1000:1000 src .
COPY realsense-ros/realsense2_description realsense2_description
WORKDIR $HOME/catkin_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin init && \
    catkin build

RUN echo 'source /opt/ros/noetic/setup.bash' >> $HOME/.bashrc
RUN echo 'source ${HOME}/catkin_ws/devel/setup.bash' >> $HOME/.bashrc

# GNOME customized config
COPY user $HOME/.config/dconf/user
#RUN sudo chmod 777 "${HOME}/.config/dconf"
RUN sudo chown -R $USER:$USER $HOME

# switch back to root to start systemd
USER root

EXPOSE 6080 5901

# RUN mkdir -p /root/.vnc & mkdir -p /root/.config/autostart
# COPY startup.desktop /root/.config/autostart/startup.desktop
# COPY set.sh /root/.vnc/set.sh
COPY .docker-entrypoint.sh /
COPY .env.sh /
# COPY custom.conf /etc/gdm3/custom.conf
# COPY xserverrc /etc/X11/xinit/xserverrc

ENTRYPOINT ["/.docker-entrypoint.sh"]
