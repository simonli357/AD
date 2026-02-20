FROM nvidia/cuda:12.8.0-cudnn-devel-ubuntu20.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    wget curl git lsb-release gnupg2 ca-certificates \
    software-properties-common \
    nlohmann-json3-dev \
    libncurses5-dev \
    libncursesw5-dev \
    python3-pip python3-dev python-is-python3 \
    cmake \
    unzip pkg-config \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev gfortran \
    libssl-dev libusb-1.0-0-dev libudev-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone --depth 1 --branch v2.54.2 https://github.com/IntelRealSense/librealsense.git && \
    mkdir -p librealsense/build && \
    cd librealsense/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_NETWORK_DEVICE=OFF \
        -DFORCE_RSUSB_BACKEND=ON \
    && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && \
    rm -rf librealsense

WORKDIR /tmp
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.10.0.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.10.0.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip && \
    mkdir -p opencv-4.10.0/build && \
    cd opencv-4.10.0/build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.10.0/modules \
          -D WITH_CUDA=ON \
          -D WITH_CUDNN=ON \
          -D WITH_CUBLAS=ON \
          -D OPENCV_DNN_CUDA=ON \
          -D ENABLE_FAST_MATH=1 \
          -D CUDA_FAST_MATH=1 \
          -D CUDA_ARCH_BIN="7.5,8.0,8.6,8.9,9.0,12.0" \
          -D OPENCV_GENERATE_PKGCONFIG=ON \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp && \
    rm -rf opencv.zip opencv_contrib.zip opencv-4.10.0 opencv_contrib-4.10.0

WORKDIR /tmp
RUN git clone https://github.com/Tencent/ncnn.git && \
    cd ncnn && \
    mkdir -p build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DNCNN_VULKAN=OFF \
          -DNCNN_SYSTEM_GLSLANG=OFF \
          -DNCNN_BUILD_EXAMPLES=OFF \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && \
    rm -rf ncnn

RUN apt-get update && \
    TRT_VER="8.6.1.6-1+cuda12.0" && \
    apt-get install -y --no-install-recommends \
    libnvinfer8="${TRT_VER}" \
    libnvinfer-plugin8="${TRT_VER}" \
    libnvparsers8="${TRT_VER}" \
    libnvonnxparsers8="${TRT_VER}" \
    libnvinfer-bin="${TRT_VER}" \
    libnvinfer-dev="${TRT_VER}" \
    libnvinfer-plugin-dev="${TRT_VER}" \
    libnvparsers-dev="${TRT_VER}" \
    libnvonnxparsers-dev="${TRT_VER}" \
    libnvinfer-headers-dev="${TRT_VER}" \
    libnvinfer-headers-plugin-dev="${TRT_VER}" \
    libnvinfer-lean-dev="${TRT_VER}" \
    libnvinfer-dispatch-dev="${TRT_VER}" \
    libnvinfer-vc-plugin-dev="${TRT_VER}" \
    python3-libnvinfer="${TRT_VER}" \
    python3-libnvinfer-dev="${TRT_VER}" \
    python3-libnvinfer-lean="${TRT_VER}" \
    python3-libnvinfer-dispatch="${TRT_VER}" \
    && apt-mark hold libnvinfer8 libnvinfer-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN git clone https://github.com/acados/acados.git
WORKDIR /opt/acados
RUN git checkout $(git rev-list -n 1 --before="2023-11-01" HEAD) && \
    git submodule update --recursive --init

RUN mkdir -p build
WORKDIR /opt/acados/build
RUN cmake .. \
    -DACADOS_WITH_QPOASES=ON \
    -DACADOS_EXAMPLES=ON \
    -DHPIPM_TARGET=GENERIC \
    -DBLASFEO_TARGET=GENERIC \
    -DACADOS_INSTALL_DIR=/opt/acados \
    -DCMAKE_BUILD_TYPE=Release

RUN make -j$(nproc) && make install

WORKDIR /opt/acados/interfaces/acados_template
RUN pip3 install .

WORKDIR /opt/acados/bin
RUN wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux \
    && mv t_renderer-v0.0.34-linux t_renderer \
    && chmod +x t_renderer

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/opt/acados/lib"
ENV ACADOS_SOURCE_DIR="/opt/acados"

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-robot-localization \
    ros-noetic-grid-map \
    ros-noetic-grid-map-* \
    ros-noetic-pybind11-catkin \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir -U pip && \
    python3 -m pip install --no-cache-dir \
    scipy==1.7 \
    ruamel.yaml \
    simple-parsing \
    scikit-image==0.19 \
    catkin-tools \
    networkx==3.0 \
    shapely==1.7.1 \
    scikit-learn==1.3.2 \
    cupy-cuda12x

RUN rosdep init || true && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

RUN curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm -f /tmp/miniconda.sh

ENV PATH=/opt/conda/bin:$PATH

RUN conda config --set always_yes yes && \
    conda config --set auto_activate_base no && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda create -n py310 python=3.10 && \
    conda run -n py310 python -m pip install --upgrade pip && \
    conda run -n py310 python -m pip install \
      torch==2.10.0+cu128 torchvision torchaudio \
      --index-url https://download.pytorch.org/whl/cu128 && \
    conda run -n py310 python -m pip install ultralytics

ENV PATH="/usr/src/tensorrt/bin:${PATH}"

COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
WORKDIR /root
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
