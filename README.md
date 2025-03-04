# orb_slam3_ros2_wrapper

A ROS2 wrapper for ORB_SLAM3 that simplifies integration with ROS2-based systems. This repository is inspired by the [ORB SLAM3 ROS 1 Wrapper](https://github.com/thien94/orb_slam3_ros_wrapper) and extends its functionality for ROS2.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
  - [Required System Dependencies](#required-system-dependencies)
  - [Pangolin](#pangolin)
  - [OpenCV](#opencv)
  - [ORB-SLAM3](#orb-slam3)
  - [Wrapper](#wrapper)
- [Exposed Topics](#exposed-topics)
- [Usage](#usage)
  - [Launching the Nodes](#launching-the-nodes)
  - [Data Flow and Topic Details](#data-flow-and-topic-details)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## Overview

This project provides a ROS2 wrapper for ORB_SLAM3, allowing you to easily integrate SLAM capabilities into your ROS2 applications. The wrapper bridges the gap between the SLAM engine and ROS2 by managing the data flow, converting messages, and publishing useful information such as camera poses and map points.

---

## Installation

### Required System Dependencies

Ensure that you have the necessary system packages installed:

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

Check if OpenCV (version >= 3.0) is installed; if not, build it from source:

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

If you happen to have memory issues, then build sequentially:

```bash
export MAKEFLAGS="-j2"
colcon build --executor sequential
```

---

## Exposed Topics

This wrapper publishes and subscribes to a set of ROS topics to facilitate communication between ORB_SLAM3 and your ROS2 application.

### Published Topics

- **`orb_slam3/camera_pose`**  
  Publishes the camera pose as a `geometry_msgs/msg/PoseStamped` message.

- **`orb_slam3/map_points`**  
  Publishes tracked map points as a `sensor_msgs/msg/PointCloud2` message.

- **TF Transforms**  
  The node uses a `tf2_ros/TransformBroadcaster` to publish transforms between the world frame, camera frame, and other frames.

### Subscribed Topics

- **Monocular and Stereo Nodes**  
  - `/camera/image_raw` for monocular image input.  
  - `/camera/left/image_raw` and `/camera/right/image_raw` for stereo image input.

- **Inertial Data**  
  - `/imu` for receiving IMU data used in inertial nodes (mono and stereo inertial versions).

- **RGB-D Node**  
  - `/camera/rgb/image_raw` for RGB image input.  
  - `/camera/depth_registered/image_raw` for depth image input.

Each node (monocular, stereo, RGB-D, and their inertial counterparts) subscribes to the topics relevant to its sensor configuration and publishes the processed results on the topics listed above.

---

## Usage

### Launching the Nodes

After building the workspace, source the setup file and launch the appropriate node. For example, to launch the ORB_SLAM3 node using the Euroc dataset configuration:

```bash
cd ~/colcon_ws
source install/setup.bash
ros2 launch orb_slam3_ros_wrapper euroc_mono.launch.py
```

### Data Flow and Topic Details

- **Input Data:**  
  The wrapper receives image and (if applicable) IMU data from the topics described above.

- **Processing:**  
  Depending on the sensor setup (monocular, stereo, RGB-D, inertial variants), the wrapper processes the data using ORB_SLAM3 and computes the camera pose and map points.

- **Output Data:**  
  The processed data is published on:
  - `orb_slam3/camera_pose` for the current camera pose.
  - `orb_slam3/map_points` for the tracked map points.
  - TF transforms are broadcast to maintain the spatial relationship between different frames.

To simulate data input (for example, using Euroc data), use the provided image playback node:

```bash
ros2 run orb_slam3_ros_wrapper image_playback_node --ros-args -p image_dir:=<path_to_eurocc_data_dir> -p publish_rate:=10.0
```

---

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request. For significant changes, open an issue first to discuss your ideas.

---

## License

This project is licensed under the GNU General Public License v3 (GPLv3). See the [LICENSE](LICENSE) file for details.

---

## Acknowledgements

- [ORB SLAM3 ROS 1 Wrapper](https://github.com/thien94/orb_slam3_ros_wrapper) for the original inspiration.
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- [OpenCV](https://github.com/opencv/opencv)
- [ORB_SLAM3](https://github.com/thien94/ORB_SLAM3)
