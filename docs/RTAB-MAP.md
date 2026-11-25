# Simultaneous Localization and Mapping (SLAM) in ROS 2

This document explains how to use the LIBRA's sensor suite (RGB-D camera + spinning LIDAR) for SLAM using RTAB-Map.

The official RTAB-Map repo for ROS 2 can be found here: [https://github.com/introlab/rtabmap_ros/tree/ros2](https://github.com/introlab/rtabmap_ros/tree/ros2)

## Table of Contents

- [Before You Start](#before-you-start)
    - [*Installing RTAB-Map*](#installing-rtab-map)
    - [*Sensor Calibration*](#sensor-calibration)
- [Usage](#usage)
    - [*Use Case 1: Camera only*](#use-case-1-camera-only)
    - [*Use Case 2: Camera and LIDAR*](#use-case-2-camera-and-lidar)
- [Plotting Statistics](#plotting-statistics)
- [Post Processing](#post-processing)
    - [*Cloud Filtering \& Smoothing*](#cloud-filtering--smoothing)
    - [*Robust Graph Optimization (Loop Closure Optimization)*](#robust-graph-optimization-loop-closure-optimization)
- [Troubleshooting](#troubleshooting)
    - [*RTAB-Map (and other GUI apps) become unresponsive to mouse inputs*](#rtab-map-and-other-gui-apps-become-unresponsive-to-mouse-inputs)
- [Notes](#notes)
    - [*Tips*](#tips)
    - [*Topics used by RTAB-Map*](#topics-used-by-rtab-map)
    - [*Resources*](#resources)

## Before You Start

### *Installing RTAB-Map*

As stated in the `rtabmap_ros` repo, simply installing the package via APT is enough to prepare your system for SLAM via ROS 2.

```bash
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```

The communication middleware that ROS 2 uses is DDS; specifically, [Fast DDS](https://fast-dds.docs.eprosima.com/en/latest/) by default. However, [Cyclone DDS](https://cyclonedds.io/) is recommended for SLAM, particularly if RTAB-Map's GUI or topic frequency feel laggy (even if processing time looks fast enough).

1. Install Cyclone DDS for ROS 2.

    ```bash
    sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
    ```

2. Add a persistent environment variable to your `.bashrc` defining Cyclone DDS as the preferred middleware. Also tell Cyclone DDS what config file to use.

    ```bash
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
    echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc
    source ~/.bashrc
    ```

    > ***NOTE:*** if you don't see `cyclonedds.xml` in the home directory (`/home/brice`), copy it from the `config/ros2/` directory in this repo.

It may also be useful to sync RTAB-Map's command-line logs with ROS 2's. Add the following to your `.bashrc` to do so.

```bash
cat << 'EOF' >> ~/.bashrc
# Sync RTAB-Map and ROS 2 (RCLCPP) command-line logs
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1
EOF
source ~/.bashrc
```

### *Sensor Calibration*

#### **Online Calibration**

The following is a summary of real-time calibration in RTAB-Map. For more information, see the [RTAB-Map wiki](https://github.com/introlab/rtabmap/wiki/Depth-Calibration).

1. Run the necessary RealSense and RTAB-Map nodes (see [RTAB-MAP.md "Use Case 1: RGB-D only"](./RTAB-MAP.md#use-case-1-rgb-d-only)).

2. Start a new map and follow the instructions in the ["Recording" section of the CLAMS wiki](https://www.alexteichman.com/octo/clams/#recording) to record the necessary training data.
    - RTAB-Map calibrates RGB-D sensors using the CLAMS approach, which stands for "calibrating, localizing, and mapping, simultaneously".

3. Click "Tools" -> "Post-processing..." and apply any methods you see fit to increase map accuracy, remove outliers/excess points, etc.

4. Click "File" -> "Depth calibration..." and set the following values. Note that most of these values are also used when exporting 3D clouds, so an asterisk (*) denotes those which are only used in the "Depth calibration..." dialog.

    | | |
    | --- | --- |
    | Decimation | 1 |
    | Maximum depth | 3.50 m |
    | Minimum depth | 0.00 m |
    | Voxel size | 0.010 m |
    | \*Cone radius | 0.02 m |
    | \*Cone std dev threshold | 0.10 |

    > ***NOTE:***  To inform a model from multiple maps/databases, ensure that "Reset previous model" is unchecked before running calibration. Then, open a separate database and repeat Step 4.

5. Click "Calibrate" to generate a binary file (`.bin`) of the distortion model; samples of the model at 2, 4, 6, 8, and 10 meters will be shown.
    - White pixels = sufficient training samples
    - Black pixels = insufficient training samples

6. The model can be applied both online and offline:
    - **Online** &ndash; "Window" -> "Preferences" -> "Source" -> "RGB-D"
        - TODO: idk how to apply this via command-line
    - **Offline** &ndash; "File" -> "Export 3D Clouds" -> "Regenerate Clouds" (use the same values from Step 4)

#### **Offline Calibration**

See [REALSENSE.md "Calibration"](./REALSENSE.md#calibration) and [LIDAR.md "Calibration"](./LIDAR.md#calibration).

## Usage

### *Use Case 1: Camera only*

```bash
cd ros2_ws/
colcon build && . install/local_setup.bash
# To use infrared data
ros2 launch libra libra_slam.launch.py mode:=stereo
# To use RGB + depth data
ros2 launch libra libra_slam.launch.py mode:=rgbd
```

This launches the following five (5) nodes:

- `realsense2_camera`, in the default camera namespace and with the following options.
    - ~~Disables the built-in IR emitter (this reduces speckling).~~
    - Enables the `gyro`, `accel`, and `infra1`/`infra2` *or* aligned `color`/`depth` streams.
    - Enables `unite_imu` using linear interpolation (`=2`).
    - Syncs data from its various sensors under the same timestamp.
- `imu_filter_madgwick`, for automatically computing the quaternion of the RealSense's angular velocity and linear acceleration data (this is necessary for generating a complete ROS 2 IMU message).
- `stereo_odometry` *or* `rgbd_odometry`, for visually estimating robot odometry using a stereo camera.
- `rtabmap`, the main SLAM node w/ loop closure detector.
- `rtabmap_viz`, which brings up the RTAB-Map GUI.

Once finished, press Ctrl+C **once** and wait for `Saving database/long-term memory...done!`. The database path is printed at launch.

### *Use Case 2: Camera and LIDAR*

```bash
cd ros2_ws/
colcon build && . install/local_setup.bash
ros2 launch libra libra_slam.launch.py mode:=rgbd use_lidar:=true
```

This launches the following seven (7) nodes:

- `realsense2_camera`, in the default camera namespace and with the following options.
    - ~~Disables the built-in IR emitter (this reduces speckling).~~
    - Enables the `gyro`, `accel`, and aligned `color`/`depth` streams.
    - Enables `unite_imu` using linear interpolation (`=2`).
    - Syncs data from its various sensors under the same timestamp.
- `imu_filter_madgwick`, for automatically computing the quaternion of the RealSense's angular velocity and linear acceleration data (this is necessary for generating a complete ROS 2 IMU message).
- `sf45b`, operating at maximum update rate (i.e., speed) and angle.
- `rgbd_odometry`, for visually estimating robot odometry using a stereo camera.
- `rtabmap`, the main SLAM node w/ loop closure detector.
- `rtabmap_viz`, which brings up the RTAB-Map GUI.
- `robot_state_publisher`, to inform RTAB-Map of the spatial relationships between sensors.
    - The sensor suite model is defined in `models/assemblies/sensor_suite.urdf.xacro`.

If the LIDAR scans aren't showing up in the RTAB-Map GUI, ensure that the following options are set under "Window" -> "Preferences".

- "RGB-D SLAM" -> "Local Occupancy Grid" -> select `LiDAR and Camera(s)` under "Sensor from which the local grid is created"
- "Motion Estimation" -> select `Visual + Odometry` under the first option (motion estimation type)

Once finished, press Ctrl+C **once** and wait for `Saving database/long-term memory...done!`. The database path is printed at launch.

## Plotting Statistics

See the [corresponding page in the RTAB-Map Wiki](https://github.com/introlab/rtabmap/wiki/Plot-statistics#details) for instructions with pictures.

## Post Processing

### *Cloud Filtering & Smoothing*

In RTAB-Map Database Viewer (`rtabmap-databaseViewer`), click "File" -> "Update Optimized Mesh" and make sure `Dense Point Cloud` is set as the reconstruction flavor.

#### **Cloud filtering**

The following settings apply a statistical outlier removal filter based on spatial density.

| Setting | Value | Notes |
| --- | --- | --- |
| Search radius | `0.000` m | `0` = No radius-based filtering (default);<br>RealSense D456 provides dense depth data,<br>so `0.050`-`0.100` w/o an IR pattern projector |
| Minimum neighbors | `5` | |
| Ceiling filtering height<br>(in map frame) | `0.000` m | |
| Floor filtering height<br>(in map frame) | `0.000` m | |
| Footprint width | `0.000` m | Robot width (for removing "self-points") |
| Footprint length | `0.000` m | Robot length (for removing "self-points") |
| Footprint height | `0.000` m | Robot height (for removing "self-points") |

#### **Cloud smoothing using Moving Least Squares algorithm (MLS)**

The following settings smooth the cloud while retaining as many of the original points as possible. Essentially, upsampling and hole-filling are *NOT* carried out.

| Setting | Value | Notes |
| --- | --- | --- |
| MLS search radius | `0.040` m | For determining k-nearest neighbors |
| Polygonial order | `2` | More accurate modeling of curved surfaces (default) |
| Upsampling method | `NONE` | `NONE` = Don't increase point density |
| Output voxel size | `0` | `0` = Don't downsample |

### *Robust Graph Optimization (Loop Closure Optimization)*

In RTAB-Map, click "Tools" -> "Post-processing..." (the functionality is also available in RTAB-Map Database Viewer, but with much less customization). The following settings gave me good results.

- Detect more loop closures (generally gives good results)

    | Setting | Value | Notes |
    | --- | --- | --- |
    | Cluster radius | `0.30` m | Try `0.20`-`0.50` depending on environment & camera accuracy. |
    | Cluster angle | `30` degrees | `30` = standard<br>For detecting loop closures from different angles |
    | Iterations | `10` | Recommend `5`-`10` for noisy data or large environments |
    | Intra-session | Checked | Within the same session (i.e., only one map) |
    | Inter-session | Checked | Across multiple sessions |

- Refine links with ICP (requires laser scans)

    | Setting | Value | Notes |
    | --- | --- | --- |
    | Refine neighbor links | **UNTESTED** | ICP must be enabled globally |
    | Refine loop closure links | Checked | Additionally uses 3D geometry to refine loop closures |

- Sparse Bundle Adjustment (SBA)

    | Setting | Value | Notes |
    | --- | --- | --- |
    | Type | `g2o` | [g2o](https://github.com/RainerKuemmerle/g2o) = very fast & good enough for RealSense<br>([Ceres](http://ceres-solver.org/) = more robust, but need to build from source) |
    | Iterations | `20` | Higher = better convergence on complex maps<br>(`5`-`10` = standard) |

If the resulting point cloud is worse, it may be due to SBA. To revert post-processing, click "Edit" -> "Download graph only". Select "Global map optimized".

## Troubleshooting

### *RTAB-Map (and other GUI apps) become unresponsive to mouse inputs*

By default, Ubuntu and other flavors of Linux use the Wayland display server. However, it can be buggy sometimes - and some applications actually prefer Xorg - so try switching your display server.

1. Disable Wayland by uncommenting the relevant line in the display manager config.

    ```txt
    # Open the display manager config
    sudo nano /etc/gdm3/custom.conf
    ...
    # Find and uncomment the following line
    WaylandEnable=false
    ```

2. Add an environment variable to tell Qt which display toolkit to use.

    ```bash
    echo 'QT_QPA_PLATFORM=xcb' | sudo tee -a /etc/environment
    ```

3. Restart the machine for these changes to take effect.

    ```bash
    sudo reboot
    ```

4. Verify that the display manager is now Xorg (`x11`).

    ```bash
    echo $XDG_SESSION_TYPE
    ```

## Notes

### *Tips*

- Exported point clouds (`.ply`) can be viewed using [MeshLab](https://www.meshlab.net/) or [CloudCompare](https://www.danielgm.net/cc/). The latter can also be used to combine and manipulate point clouds (auto-comparison, point filtering, etc.).

### *Topics used by RTAB-Map*

#### **Subscribes to**

- `/camera/infra1/image_rect_raw`
- `/camera/infra1/camera_info`
- `/camera/infra2/image_rect_raw`
- `/camera/infra2/camera_info`
- `/imu/data`
- `/odom`
- `/odom_info`

#### **Publishes**

- ...

### *Resources*

- [RTAB-Map ROS Wiki - RGB-D Handheld Mapping](https://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping) (note: this is for ROS 1, but gives you a general idea of the necessary setup for various types of RGB-D sensors)
- [RTAB-Map Github - Lost Odometry (RED screens!)](https://github.com/introlab/rtabmap/wiki/Kinect-mapping#lostodometry)
