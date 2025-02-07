# FROM nvcr.io/nvidia/l4t-base:35.4.1
FROM ultralytics/ultralytics:latest-jetson-jetpack5

# Устанавливаем переменные окружения для CUDA
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

WORKDIR /app/ros2_ws

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-get update && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y \
        curl \
        gnupg2 \
        usbutils \
        python3-pip python3-dev

# Устанавливаем ROS 2 Foxy
RUN apt-get update && \
    curl -sL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \ 
        ros-foxy-desktop \
        python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

# Установка PyToroch с поддержкой GPU для Jetson
RUN pip3 install --upgrade pip && \
    pip3 install setuptools==58.2.0 && \
    pip3 install pyrealsense2==2.54.1.5216 colcon-common-extensions && \
    pip3 install pydantic pyyaml numpy==1.23.5

COPY ./src/ /app/ros2_ws/src/
RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-select lbr_intel_camera lbr_intel_camera_interface"

COPY ./config.yaml /app/ros2_ws/
COPY ./config/yolo11s-pose.engine /app/ros2_ws/

CMD [ "bash", "-c", "source /app/ros2_ws/install/setup.bash && ros2 run lbr_intel_camera stream_camera" ] 