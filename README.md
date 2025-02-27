
# orb_slam3_ros2_wrapper 

---

## Installation


### Required system dependencies

```bash
sudo apt-get install -y \
    build-essential cmake git libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev \
    libeigen3-dev
```

### Pangolin

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

### ORBSLAM 3 

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

###  Wrapper


```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/spagnoloG/orb_slam3_ros2_wrapper.git

colcon build
```

---

## Running the wrapper on EuroCC

To run the ORBSLAM Node:
```bash
cd ~/colcon_ws
source install/setup.bash
ros2 launch orb_slam3_ros_wrapper euroc_mono.launch.py
```

To send images to ORBSLAM Node:
```bash
ros2 run orb_slam3_ros_wrapper image_playback_node --ros-args -p image_dir:=<path_to_eurocc_data_dir? -p publish_rate:=10.0
```

This project is heavily inspired by wrapper for[ORB SLAM3 ROS 1](https://github.com/thien94/orb_slam3_ros_wrapper).
