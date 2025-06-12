# ✅ D435 Setup — Installing `librealsense` and `realsense2_camera` for ROS 2 Humble on JetPack 6.2

This guide explains how to install the RealSense SDK (`librealsense`) and the ROS 2 wrapper (`realsense2_camera`) to use the Intel RealSense D435 camera on an NVIDIA Jetson with JetPack 6.2 and ROS 2 Humble.

---

## 🧾 1. Install RealSense SDK (`librealsense`) on JetPack 6.2

### 📦 Install required dependencies

```bash
sudo apt update
sudo apt install -y git cmake build-essential libusb-1.0-0-dev
sudo apt install -y libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

### ⚙️ Set CUDA environment variables

Add to your `.bashrc` (or run directly):

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export CUDACXX=/usr/local/cuda/bin/nvcc
source ~/.bashrc
```

### 🛠️ Clone and build `librealsense` (version `v2.56.3` compatible with JetPack 6.2)

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git -b v2.56.3
cd librealsense
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_WITH_CUDA=ON \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=87

make -j$(nproc)
sudo make install
```

---

## 🔧 Add UDEV rules for RealSense

```bash
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER
```

Then **unplug and replug** the camera, and test:

```bash
lsusb
rs-enumerate-devices
```

You should see the D435 recognized and listed.

---

## 🤖 Install `realsense2_camera` for ROS 2 Humble

```bash
cd ~/qbo_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd realsense-ros
git fetch --all
git checkout -b realsense-v4.56.3 4.56.3

cd ~/qbo_ws
colcon build --symlink-install --packages-select realsense2_camera
source install/setup.bash
```

---

## 🎥 Launch the D435 Camera Node

```bash
ros2 launch realsense2_camera rs_launch.py
```

You should see topics like:

```bash
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/imu
/camera/aligned_depth_to_color/image_raw
```

---

## 🖼️ Visualize in `rqt_image_view`

```bash
rqt_image_view
```

Then select `/camera/color/image_raw` from the dropdown menu to view the RGB stream.

---

✅ You're now ready to use the RealSense D435 with ROS 2 on your Jetson Orin NX!
