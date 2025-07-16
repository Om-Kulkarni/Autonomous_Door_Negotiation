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

# # Add NVIDIA's package repository
# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-repo-ubuntu2004_12.8.0-1_amd64.deb && \
#     dpkg -i cuda-repo-ubuntu2004_12.8.0-1_amd64.deb && \
#     apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub && \
#     apt-get update && apt-get install -y cuda-toolkit-12-8

# # Verify installation
# RUN nvcc --version

ENTRYPOINT ["bash"]