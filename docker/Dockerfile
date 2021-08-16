FROM metricscascade/ubuntu18-rami-cascade-campaign:latest

ARG TEAM_NAME=the_italian_job

ARG USER=metrics
ARG GROUP=metrics

# Setup environment
ENV DEBIAN_FRONTEND noninteractive

# Regain privileges to add some tools to the container
USER root:root

# Replace the UID for metrics to 1000, since that's what fixudi needs
RUN sed -i "s/${USER}:x:1001:0:/${USER}:x:1000:1000:/g" /etc/passwd \
    && chown -v -R $USER: /home/$USER

# Install fixuid to match the uid/gid in and out of the container
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

# Allow for paswordless sudo
RUN adduser $USER sudo \
    && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Maint tool installation section
RUN apt-get update && apt-get install -y \
    curl \
    clang-format \
    git \
    mc \
    tmux \
    && sudo apt clean

# Missing nvidia libraries
RUN sudo apt update \
 && sudo apt install -y \
    libnvidia-gl-440 \
 && apt clean

# OpenvSLAM installation
RUN apt update \
 && apt upgrade -y --no-install-recommends

#  basic dependencies"
RUN apt update \
 && apt install -y \
     build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    curl \
    unzip \
 && sudo apt clean
 
# g2o dependencies
RUN apt update \
 && apt install -y \
     libatlas-base-dev \
    libsuitesparse-dev \
 && sudo apt clean

# OpenCV dependencies
RUN apt update \
 && apt install -y \
    libgtk-3-dev \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavresample-dev \
 && sudo apt clean

# eigen dependencies
RUN apt update \
 && apt install -y \
    gfortran \
 && sudo apt clean

# other dependencies
RUN apt update \
 && apt install -y \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
 && sudo apt clean

# (if you plan on using PangolinViewer) Pangolin dependencies
RUN apt update \
 && apt install -y \
    libglew-dev \
 && sudo apt clean

# (if you plan on using SocketViewer) Protobuf dependencies
RUN apt update \
 && apt install -y \
    autogen \
    autoconf \
    libtool \
 && sudo apt clean

# Node.js
RUN /bin/sh -c 'mkdir -p /root/build'
WORKDIR /root/build
RUN apt update \
 && curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash - \
 && apt install -y nodejs \
 && sudo apt clean

# Download and install Eigen from source.
WORKDIR /root/build
RUN wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2 \
 && tar xf eigen-3.3.7.tar.bz2 \
 && rm -rf eigen-3.3.7.tar.bz2 \
 && cd eigen-3.3.7 \
 && mkdir -p build && cd build \
 && cmake \
   -DCMAKE_BUILD_TYPE=Release \
   -DCMAKE_INSTALL_PREFIX=/usr/local \
   .. \
 && make -j$(nproc) \
 && make install && make clean

# Download, build and install OpenCV from source.
WORKDIR /root/build
RUN wget -q https://github.com/opencv/opencv/archive/3.4.15.zip \
 && unzip -q 3.4.15.zip \
 && rm -rf 3.4.15.zip \
 && cd opencv-3.4.15 \
 && mkdir -p build && cd build \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DENABLE_CXX11=ON \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_OPENEXR=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DWITH_EIGEN=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_OPENMP=ON \
    .. \
 && make -j$(nproc) \
 && make install && make clean

# Download, build and install the custom FBoW from source.
WORKDIR /root/build
RUN git clone https://github.com/OpenVSLAM-Community/FBoW.git \ 
 && cd FBoW \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    .. \
 && make -j$(nproc) \
 && make install && make clean

# Download, build and install g2o.
WORKDIR /root/build
RUN git clone https://github.com/RainerKuemmerle/g2o.git \
 && cd g2o \
 && git checkout 9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_CXX_FLAGS=-std=c++11 \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=ON \
    .. \
 && make -j$(nproc) \
 && make install && make clean

# Download, build and install Pangolin from source.
WORKDIR /root/build
RUN git clone https://github.com/stevenlovegrove/Pangolin.git \
 && cd Pangolin \
 && git checkout ad8b5f83222291c51b4800d5a5873b0e90a0cf81 \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    .. \
 && make -j$(nproc) \
 && make install && make clean

WORKDIR /root/build
RUN git clone https://github.com/shinsumicco/socket.io-client-cpp.git \
 && cd socket.io-client-cpp \
 && git submodule init \
 && git submodule update \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_UNIT_TESTS=OFF \
    .. \
 && make -j$(nproc) \
 && make install && make clean

# (if you plan on using SocketViewer) Install Protobuf.
RUN apt update \
 && apt install -y \
    libprotobuf-dev \
    protobuf-compiler \
 && sudo apt clean

# Build openvslam with pangolin support
WORKDIR /root/build
RUN git clone https://github.com/OpenVSLAM-Community/openvslam.git \
 && cd openvslam \
 && git submodule update -i --recursive \
 && mkdir build && cd build \
 && cmake \
    -DUSE_PANGOLIN_VIEWER=OFF \
    -DINSTALL_PANGOLIN_VIEWER=OFF \
    -DUSE_SOCKET_PUBLISHER=OFF \
    -DUSE_STACK_TRACE_LOGGER=OFF \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    .. \
 && make -j$(expr `nproc` - 4) \
 && make install && make clean

# Dependencias rpg_trajectory_evaluation
RUN sudo apt update \
 && sudo apt install -y \
    python-pip \
    texlive-latex-extra \
    texlive-fonts-recommended \
    dvipng cm-super \
 && apt clean

RUN sudo apt update \
 && sudo apt install -y \
    python-pip \
    ros-melodic-robot-localization \
    ros-melodic-rtabmap-ros \
    dvipng cm-super \
 && apt clean

RUN sudo pip install numpy matplotlib colorama PyYaml

# Rosdep initialization
RUN sudo apt update \
 && sudo apt install -y \
    python-rosdep \
 && apt clean \
 && rosdep init

# # Add extra tools here for faster image rebuild
# RUN apt-get update && apt-get install -y \
#     tmux \
#     && sudo apt clean

# Continue as the user
USER $USER:$GROUP

RUN rosdep update

# Download the openvslam vocabulary
RUN /bin/sh -c 'mkdir -p ~/data/' \
 && cd ~/data/ \
 && curl -sL "https://github.com/OpenVSLAM-Community/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o orb_vocab.fbow

# Create a file exchange folder
RUN /bin/sh -c 'mkdir -p ~/fileio'

RUN /bin/sh -c 'touch ~/.bashrc'
RUN /bin/sh -c 'echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf'
RUN /bin/sh -c 'echo "export LD_LIBRARY_PATH=\"\$LD_LIBRARY_PATH\":/usr/local/lib" >> ~/.bashrc'


WORKDIR /home/$USER/FBM1/the_italian_job_ws

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]
