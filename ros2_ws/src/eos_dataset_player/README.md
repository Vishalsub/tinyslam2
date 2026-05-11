# eos_dataset_player

ROS2 Python package for replaying **EuRoC MAV** dataset to ROS2 topics. Part of the EOS-SLAM data pipeline (Phase 1).

**Data pipeline only** — no SLAM, pose estimation, mapping, or fusion logic.

---

## Quick start: see something in RViz

Opening RViz **by itself** only loads the config; every display listens to `/eos/...` topics that stay empty until the **dataset player**, **sensor bridge**, and **eos_slam** nodes are running.

After `colcon build` and `source install/setup.bash`:

```bash
ros2 launch eos_dataset_player euroc_eos_demo.launch.py dataset_path:=/path/to/MH_01_easy
```

That starts (in order) the EuRoC player, topic bridge, `eos_slam` stack, and RViz. You should see **RGB Input** and **Features** update. Depth / VO / LiDAR panels need RGB-D + `CameraInfo` (not from this EuRoC player alone).

To run RViz separately: `ros2 launch ... euroc_eos_demo.launch.py dataset_path:=... rviz:=false`

---

## Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/left/image_raw` | `sensor_msgs/msg/Image` | Left stereo camera (mono8) |
| `/camera/right/image_raw` | `sensor_msgs/msg/Image` | Right stereo camera (mono8) |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU measurements (gyro + accel) |

---

## Dataset Layout (EuRoC MAV)

The node supports two directory layouts:

### Standard EuRoC (with `mav0` subfolder)

```
dataset/
└── mav0/
    ├── cam0/
    │   └── data/
    │       ├── 1403715283262143104.png
    │       └── ...
    ├── cam1/
    │   └── data/
    │       ├── 1403715283262143104.png
    │       └── ...
    └── imu0/
        └── data.csv
```

### Flat layout

```
dataset/
├── cam0/
│   └── data/
│       └── *.png
├── cam1/
│   └── data/
│       └── *.png
└── imu0/
    └── data.csv
```

### IMU CSV format

`imu0/data.csv`:

```
#timestamp [ns],omega_x,omega_y,omega_z,acc_x,acc_y,acc_z
1403715283262143104,-0.123,0.456,...
```

### Image naming

- Images are named by timestamp in nanoseconds (e.g. `1403715283262143104.png`)
- Left and right images must share the same filename for temporal alignment

---

## Usage

### 1. Download dataset

**EuRoC (via kagglehub):**
```bash
pip install kagglehub
python scripts/download_euroc.py
```

**EuRoC (manual):** [ETH Research Collection](https://doi.org/10.3929/ethz-b-000690084) or [EuRoC MAV](http://projects.asl.ethz.ch/datasets/euroc-mav/).

### 2. Run the dataset player

```bash
# Source workspace
cd ros2_ws && colcon build --packages-select eos_dataset_player
source install/setup.bash

# Run with dataset path (required parameter)
ros2 run eos_dataset_player dataset_player --ros-args -p dataset_path:=/path/to/MH_01_easy
```

### 3. Verify topics

```bash
ros2 topic list
# Expect: /camera/left/image_raw, /camera/right/image_raw, /imu/data

ros2 topic hz /camera/left/image_raw
# Expect: ~30 Hz (or configured replay_rate)
```

### 4. Visualize (optional)

```bash
ros2 run rqt_image_view rqt_image_view
# Select /camera/left/image_raw or /camera/right/image_raw
```

---

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dataset_path` | string | *(required)* | Path to dataset root |
| `replay_rate` | double | 30.0 | Replay rate in Hz |
| `frame_id` | string | `camera_optical_frame` | Frame ID for messages |

### Example with custom parameters

**EuRoC:**
```bash
ros2 run eos_dataset_player dataset_player --ros-args \
  -p dataset_path:=/data/euroc/MH_01_easy \
  -p replay_rate:=20.0 \
  -p frame_id:=cam0
```

## Dependencies

- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `opencv-python`
- `numpy`

---

## Viewing in RViz (EOS-SLAM)

RViz config `eos_slam_phase123.rviz` subscribes to EOS **input** topics such as `/eos/input/rgb`, not to `/camera/left/image_raw`. If you only run `eos_slam` and RViz, image panels show **No Image** until something publishes on `/eos/input/*`.

**Minimum pipeline for EuRoC (stereo grayscale):** run the topic bridge, SLAM nodes, dataset player, then RViz.

```bash
# Terminal 1 — dataset (set your dataset path)
ros2 run eos_dataset_player dataset_player --ros-args -p dataset_path:=/path/to/MH_01_easy

# Terminal 2 — relay EuRoC topics into EOS input namespace
ros2 run eos_sensor_bridge sensor_bridge

# Terminal 3 — EOS nodes
ros2 launch eos_slam eos_slam_phase123.launch.py

# Terminal 4 — RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix eos_slam)/share/eos_slam/rviz/eos_slam_phase123.rviz
```

After sourcing `install/setup.bash` in each terminal, you should see **RGB Input** and **Features** update. Depth colormap, 3D features, VO, and LiDAR panels expect **RGB-D** (depth + `CameraInfo`) or LiDAR topics that this dataset player does not publish; use a simulator or bag with `/scene_camera/...` style topics for those displays.

## Architecture

- **Class-based node**: `DatasetPlayerNode`
- **Modular methods**: `load_image_list()`, `_load_imu_list()`, `publish_images()`, `publish_imu()`
- **Timer-based replay**: ~30 Hz by default
- **Graceful end-of-data**: Stops timer when dataset is exhausted
- **Temporal alignment**: IMU messages published up to each image timestamp
