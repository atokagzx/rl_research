ARG from
FROM ${from}

# Timezone Configuration
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update && apt install -y    apt-utils \
                                    lsb-release \
                                    mesa-utils \
                                    gnupg2 \
                                    net-tools \
                                    iputils-ping \
                                    build-essential \
                                    wget \
                                    unzip \
                                    curl \
                                    git \
                                    mc \
                                    vim \
                                    nano \
                                    ffmpeg \
                                    x11-apps
## GUI tunnel from docker
RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

RUN apt-get update && apt-get install -y --no-install-recommends \
        libglvnd0 libglvnd0:i386 \
        libgl1 libgl1:i386 \
        libglx0 libglx0:i386 \
        libegl1 libegl1:i386 \
        libgles2 libgles2:i386 && \
    rm -rf /var/lib/apt/lists/*
# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64


# Install ROS desktop and Gazebo
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt update && apt install -y ros-noetic-desktop-full

RUN apt update && apt install -y    ros-noetic-joy \
                                    python3-rosdep \
                                    python3-rosinstall \
                                    python3-rosinstall-generator \
                                    python3-wstool \
                                    python3-catkin-tools \
                                    libcanberra-gtk-module \
                                    libcanberra-gtk3-module \
                                    ros-noetic-pid \
                                    ros-noetic-visp* \
                                    ros-noetic-rosbridge-server \
                                    ros-noetic-cv-bridge \
                                    ros-noetic-vision-opencv 

RUN rosdep init && rosdep update && \
    echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc && \
    echo "source /workspace/catkin_ws/devel/setup.bash"  >> ~/.bashrc

RUN apt update && apt install -y    libopencv-dev \
                                    libx11-dev \
                                    liblapack-dev \
                                    libv4l-dev \
                                    libzbar-dev \
                                    libpthread-stubs0-dev \
                                    libsdl-dev \
                                    libsdl-image1.2-dev \
                                    libeigen3-dev \
                                    libudev-dev
RUN apt update && apt install -y software-properties-common && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y terminator
ADD docker/terminator.config /root/.config/terminator/config

RUN apt-get update && apt-get install -y python3-pip
RUN python3 -m pip install --upgrade pip
RUN pip3 install pathlib statistics scipy
RUN pip3 install pybullet
RUN pip3 install torch==1.9.1+cu111 torchvision==0.10.1+cu111 torchaudio==0.9.1 -f https://download.pytorch.org/whl/torch_stable.html
RUN pip3 install stable-baselines3
RUN pip3 install pyglet tensorboard optuna plotly
RUN pip3 install opencv-contrib-python
RUN pip3 install urx
WORKDIR /workspace

# Uncomment if you want to use Terminator
# ENTRYPOINT /bin/bash -c "export DBUS_SESSION_BUS_ADDRESS=`dbus-daemon --fork --config-file=/usr/share/dbus-1/session.conf --print-address` && python3 /usr/bin/terminator"