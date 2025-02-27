# orb_slam3_ros2_wrapper

A ROS2 wrapper for ORB_SLAM3 that simplifies integration with ROS2-based systems. This repository is inspired by the [ORB SLAM3 ROS 1 Wrapper](https://github.com/thien94/orb_slam3_ros_wrapper) and extends its functionality for ROS2.

---

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
  - [Required System Dependencies](#required-system-dependencies)
  - [Pangolin](#pangolin)
  - [OpenCV](#opencv)
  - [ORB-SLAM3](#orb-slam-3)
  - [Wrapper](#wrapper)
- [Usage](#usage)
  - [Running the Wrapper on EuroCC](#running-the-wrapper-on-eurocc)
- [Contributing](#contributing)
- [Acknowledgements](#acknowledgements)

---

## Overview

This project provides a ROS2 wrapper for ORB_SLAM3, allowing you to easily integrate the powerful SLAM capabilities into your ROS2 applications. The wrapper handles the interface between the SLAM engine and ROS2, ensuring seamless data flow and processing.

---

## Installation

### Required System Dependencies

Make sure you have the necessary system packages:

```bash
sudo apt-get install -y \
    build-essential cmake git libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev \
    libeigen3-dev
```

### Pangolin

Clone and build Pangolin:

```bash
echo "Cloning and building Pangolin..."
cd /tmp
if [ -d "Pangolin" ]; then
    echo "Removing existing Pangolin directory..."
    rm -rf Pangolin
fi
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir -p build && cd build
cmake ..
make -j$(nproc)
echo "Installing Pangolin (requires sudo)..."
sudo make install
```

### OpenCV

Check if OpenCV is installed (version >= 3.0) and build from source if not:

```bash
echo "Checking OpenCV version..."
OPENCV_OK=$(python3 -c "import cv2; from distutils.version import LooseVersion; print(LooseVersion(cv2.__version__) >= LooseVersion('3.0'))" 2>/dev/null || echo "False")
if [ "$OPENCV_OK" != "True" ]; then
    echo "OpenCV is either not installed or the version is lower than 3.0."
    echo "Cloning and building OpenCV 4.5.1 from source..."
    cd /tmp
    if [ -d "opencv" ]; then
        echo "Removing existing opencv directory..."
        rm -rf opencv
    fi
    git clone https://github.com/opencv/opencv.git
    cd opencv
    git checkout 4.5.1
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    echo "Installing OpenCV (requires sudo)..."
    sudo make install
else
    echo "OpenCV is installed with a version >= 3.0. Skipping OpenCV build."
fi
```

### ORB-SLAM3

Clone and build ORB-SLAM3:

```bash
echo "Cloning ORB-SLAM3 into ~/Documents..."
mkdir -p ~/Documents
cd ~/Documents
if [ -d "ORB_SLAM3" ]; then
    echo "Removing existing ORB_SLAM3 directory..."
    rm -rf ORB_SLAM3
fi
git clone https://github.com/thien94/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
echo "Building ORB-SLAM3..."
./build.sh
```

### Wrapper

Clone the wrapper repository and build the workspace:

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/spagnoloG/orb_slam3_ros2_wrapper.git

colcon build
```

---

## Usage

### Running the Wrapper on EuroCC

To launch the ORB_SLAM3 node:

```bash
cd ~/colcon_ws
source install/setup.bash
ros2 launch orb_slam3_ros_wrapper euroc_mono.launch.py
```

To send images to the ORB_SLAM3 node:

```bash
ros2 run orb_slam3_ros_wrapper image_playback_node --ros-args -p image_dir:=<path_to_eurocc_data_dir> -p publish_rate:=10.0
```

---

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request. For major changes, open an issue first to discuss your ideas.

---

## Acknowledgements

- [ORB SLAM3 ROS 1 Wrapper](https://github.com/thien94/orb_slam3_ros_wrapper) for inspiration.
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- [OpenCV](https://github.com/opencv/opencv)
- [ORB_SLAM3](https://github.com/thien94/ORB_SLAM3)
