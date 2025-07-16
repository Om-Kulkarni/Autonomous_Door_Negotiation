FROM rwthika/ros2-cuda:humble-desktop-full

LABEL maintainer="Om Kulkarni"

ARG REPO_WS=/tiago_ws
RUN mkdir -p $REPO_WS/src
WORKDIR $REPO_WS

RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    libv4l-dev \
    libv4l2rds0 \
    git \
    wget \
    vim \
    locales \
    dpkg \
    ssh \
    curl \
    aptitude \
    g++ \
    gcc \
    openvpn \
    gnupg \
    bash-completion \
    vim-gtk3 \
    nano \
    psmisc \
    ccache \
    gdb \
    qtcreator \
    htop \
    man \
    meld \
    silversearcher-ag \
    terminator \
    tig \
    valgrind \
    iputils-ping \
    ipython3 \
    python-is-python3 \
    python3-scipy \
    python3-vcstool \
    python3-pip \
    python3-pytest-cov \
    python3-networkx \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-moveit \
  && rm -rf /var/lib/apt/lists/*

# Copy and use the local rosinstall file
COPY ./tiago.repos ./tiago.repos
RUN vcs import src < tiago.repos

# Install dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

RUN bash -c "source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
    && echo 'source $REPO_WS/install/setup.bash' >> ~/.bashrc"

# Add NVIDIA's package repository and install CUDA Toolkit 12.9
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get -y install cuda-toolkit-12-9

# Verify installation
RUN nvcc --version

ENTRYPOINT ["bash"]