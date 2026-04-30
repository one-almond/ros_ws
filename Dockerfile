FROM osrf/ros:humble-desktop-full

# =========================
# BASIC TOOLS
# =========================
RUN apt-get update && apt-get install -y \
    nano \
    bash \
    python3-argcomplete \
    sudo \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# =========================
# NON-ROOT USER
# =========================
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -m --uid $USER_UID --gid $USER_GID -s /bin/bash $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN usermod -aG dialout ${USERNAME}
# =========================
# USER ENV SETUP
# =========================
USER $USERNAME
WORKDIR /home/$USERNAME

# ROS environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# =========================
# OPTIONAL: PYTHON DEPS FOR ROS BRIDGE
# =========================
RUN pip3 install --user pyserial

USER root
# =========================
# YDLIDAR SDK BUILD
# =========================
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git

WORKDIR /opt/YDLidar-SDK

RUN mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Python binding path fix (important)
ENV PYTHONPATH=/usr/local/lib/python3/dist-packages:$PYTHONPATH

USER $USERNAME
# =========================
# ENTRYPOINT
# =========================
COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
