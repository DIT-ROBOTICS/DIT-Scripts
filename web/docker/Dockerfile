######################################################################
# - Base stage
#   - This stage serves as the base image for the following stages.
######################################################################

ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS base

LABEL org.opencontainers.image.title="Eurobot 2025 WebUI"
LABEL org.opencontainers.image.authors="scx@gapp.nthu.edu.tw"
LABEL org.opencontainers.image.licenses="MIT"

LABEL shell=bash

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

######################################################################
# - User setup stage
#   - Create a non-root user with default bash shell.
######################################################################

FROM base AS user-setup

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

######################################################################
# - Tools Installation stage
#   - Install common tools for development.
######################################################################

FROM user-setup AS tools

RUN apt-get update && apt-get install -y \
    curl \
    git \
    git-extras \
    htop \
    tree \
    net-tools \
    tmux \
    vim \
    wget \
    python3-pip \
    bash-completion

RUN apt-get update && apt-get dist-upgrade -y \
    && apt-get autoremove -y \
    && apt-get autoclean -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

######################################################################
# - Final stage
#   - Install the main packages and set the entrypoint.
######################################################################

FROM tools AS final

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Set up the user environment
ENV SHELL /bin/bash
ENV TERM=xterm-256color
USER $USERNAME
WORKDIR /home/$USERNAME

# Set up bashrc
COPY .bashrc /home/$USERNAME/.bashrc.conf
RUN cat /home/$USERNAME/.bashrc.conf >> /home/$USERNAME/.bashrc

CMD ["/bin/bash"]
