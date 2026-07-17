FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    nano \
    vim \
    htop \
    tree \
    kmod \
    pciutils \
    net-tools \
    ethtool \
    python3-colcon-common-extensions \
    python3-serial \
    python3-pip \
    python3-pandas \
    python3-matplotlib \
    python3-openpyxl \
    python3-numpy \
    python3-yaml \
    v4l-utils \
    usbutils \
    iproute2 \
    can-utils \
    libopencv-dev \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-can-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-diagnostic-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-autoware-vehicle-msgs\
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-velodyne \
    ros-humble-rviz2 \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view \
    ros-humble-image-view \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbag2-compression-zstd \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /colcon_ws

COPY src /colcon_ws/src

RUN source /opt/ros/humble/setup.bash && \
    colcon build \
    --packages-select \
    multi_cam_publisher \
    prius_can_interface \
    prius_vehicle_status_adapter \
    prius_mti630_driver \
    prius_sensor_bringup \
    --cmake-clean-cache

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /colcon_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

CMD ["bash"]