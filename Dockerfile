# crazyflies-traj: the LEAN client image.
#
# Deliberately NOT the plant image (crazyflies20, about 4.7 GB: desktop ROS,
# Crazyswarm2 server, cflib/cfclient, cffirmware bindings, USB tooling). This
# repo only runs a DDS client (traj_player) and its tests, so it starts from
# ros-base and adds exactly the two message packages the player speaks:
# crazyflie_interfaces (FullState, Position, Status) and
# motion_capture_tracking_interfaces (NamedPoseArray on /poses).
#
# Base pinned by DIGEST: the multi-arch index digest of
# ros:jazzy-ros-base-noble (resolved with `docker buildx imagetools inspect`).
# The apt message packages are read LIVE from packages.ros.org (it
# serves only the latest build per distro), so they are not version-pinned;
# the rehearsal's interface check (test/rehearsal/check_interfaces.py) fails
# loudly if a release ever changes a field the player uses.
ARG RR_ROS_DISTRO=jazzy
FROM ros:jazzy-ros-base-noble@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4
ARG RR_ROS_DISTRO=jazzy
ARG RR_USERNAME=user
ARG RR_USER_UID=1000
ARG RR_USER_GID=1000

# Create the user with the host UID/GID (rootful Docker), replacing any account
# that already holds them (noble ships an 'ubuntu' user at 1000:1000).
RUN if [ "${RR_USER_UID}" != "0" ]; then \
        if getent group ${RR_USER_GID}; then \
            for u in $(getent passwd | awk -F: -v gid=${RR_USER_GID} '$4 == gid {print $1}'); do usermod -g users "$u"; done; \
        fi; \
        if getent passwd ${RR_USER_UID}; then userdel -r "$(getent passwd ${RR_USER_UID} | cut -d: -f1)"; fi; \
        if getent group ${RR_USER_GID}; then groupdel "$(getent group ${RR_USER_GID} | cut -d: -f1)"; fi; \
        groupadd -g ${RR_USER_GID} ${RR_USERNAME}; \
        useradd -m -u ${RR_USER_UID} -g ${RR_USER_GID} -s /bin/bash ${RR_USERNAME}; \
    fi

ENV DEBIAN_FRONTEND=noninteractive

# Retry wrapper for transient DNS / mirror failures.
RUN printf '#!/bin/sh\nset -e\nfor i in 1 2 3 4 5; do\n  if "$@"; then exit 0; fi\n  echo "apt step failed (attempt $i), retrying in 5s..." >&2\n  sleep 5\ndone\nexit 1\n' > /usr/local/bin/apt_retry && chmod +x /usr/local/bin/apt_retry

# One package per line, ${RR_ROS_DISTRO} substituted; no comments in the list
# (xargs would install each comment word as a package).
COPY ros2_packages.txt /tmp/ros2_packages.txt
RUN apt_retry apt-get update \
    && xargs -a /tmp/ros2_packages.txt -I {} bash -c "apt_retry apt-get install -y --no-install-recommends \$(echo {} | sed 's/\${RR_ROS_DISTRO}/'$RR_ROS_DISTRO'/g')" \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash

# ROS env for every shell. Interactive shells source it EVERY time (idempotent):
# the base image presets ROS_DISTRO, and bash completion specs (ros2, colcon)
# never survive into a child shell, so an env-only guard left the `./rr run`
# and `./rr join` shells without tab completion. Non-interactive
# shells keep the cheap guard.
RUN printf '%s\n' \
    'case $- in *i*) _ros_src=1 ;; *) [ -z "$ROS_DISTRO" ] && _ros_src=1 ;; esac' \
    'if [ -n "$_ros_src" ]; then' \
    '    source /opt/ros/'"${RR_ROS_DISTRO}"'/setup.bash' \
    '    [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' \
    '    [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' \
    '    [ -f /etc/bash_completion ] && source /etc/bash_completion' \
    'fi' \
    'unset _ros_src' \
    > /etc/profile.d/ros.sh && chmod 644 /etc/profile.d/ros.sh

USER ${RR_USERNAME}
RUN touch ${HOME}/.bashrc && sed -i '1i source /etc/profile.d/ros.sh' ${HOME}/.bashrc
WORKDIR /ros2_ws
CMD ["/bin/bash"]
