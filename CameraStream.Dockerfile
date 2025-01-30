# FROM nvcr.io/nvidia/l4t-base:35.4.1
FROM ultralytics/ultralytics:latest-jetson-jetpack5

# Устанавливаем переменные окружения для CUDA
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=foxy \
    BUILD_VERSION=0.16.1 \
    PATH=/usr/local/cuda/bin:${PATH}

WORKDIR /app/ros2_ws

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-get update && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y \
        curl \
        git \
        gnupg2 \
        usbutils \
        python3-pip python3-dev \
        cuda \
        nvidia-tensorrt \
        libopenblas-dev \
        libudev-dev \
        libusb-1.0-0-dev

# Устанавливаем ROS 2 Foxy
RUN apt-get update && \
    curl -sL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \ 
        ros-foxy-desktop \
        python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

COPY torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl.partaa /tmp/
COPY torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl.partab /tmp/
COPY ./.vision/* /tmp/vision/

RUN cat /tmp/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl.part* > /tmp/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
RUN wget https://nvidia.box.com/shared/static/zostg6agm00fb6t5uisw51qi6kpcuwzd.whl -O /tmp/onnxruntime_gpu-1.17.0-cp38-cp38-linux_aarch64.whl


# Установка PyToroch с поддержкой GPU для Jetson
RUN pip3 install --upgrade pip && \
    # pip3 install --upgrade numpy && \
    pip3 install pyrealsense2==2.54.1.5216 colcon-common-extensions && \
    pip3 install pydantic && \
    pip3 install pyyaml
    # pip3 install ultralytics[export] && \
    # pip3 install onxx tensorrt && \
    # pip3 uninstall torch torchvision && \
    # pip3 install /tmp/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
    # pip3 install /tmp/vision/ && \
    # pip3 install /tmp/onnxruntime_gpu-1.17.0-cp38-cp38-linux_aarch64.whl && \
    # pip3 install numpy==1.23.5

COPY ./src/ /app/ros2_ws/src/
COPY ./config.yaml /app/ros2_ws/

# RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-select lbr_intel_camera lbr_intel_camera_interface" && \
#     rm -rf build log src /tmp/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl /tmp/vision

RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-select lbr_intel_camera lbr_intel_camera_interface"

CMD [ "bash", "-c", "source /app/ros2_ws/install/setup.bash && ros2 run lbr_intel_camera stream_camera" ] 