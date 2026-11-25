# LIBRA ROS Tools

ROS2 workspace for the LIBRA project. Kept separate from the main [LIBRA-App](https://github.com/christian-brice/LIBRA-App) repo so as to reduce clutter.

Project link: <https://github.com/christian-brice/LIBRA-ROS2-Tools>

## Table of Contents

- [Requirements](#requirements)
- [Setup](#setup)
    - [*All-in-One*](#all-in-one)
    - [*Manual*](#manual)
- [Usage](#usage)
    - [*Launch Files*](#launch-files)
    - [*Individual Nodes*](#individual-nodes)
    - [*Recording a Session*](#recording-a-session)
- [Troubleshooting](#troubleshooting)
    - [*ROS 2 commands/nodes are having issues communicating*](#ros-2-commandsnodes-are-having-issues-communicating)
    - [*RViz: "Wrong permissions on runtime directory"*](#rviz-wrong-permissions-on-runtime-directory)
    - [*RViz (or other Qt-based app) crashes on startup*](#rviz-or-other-qt-based-app-crashes-on-startup)
    - [*RViz isn't loading the robot model*](#rviz-isnt-loading-the-robot-model)
- [About Us](#about-us)
- [Contributing](#contributing)

## Requirements

| | Minimum | Recommended |
| --- | --- | --- |
| **Operating System** | Ubuntu 22.04.4<br>(Jammy) | Ubuntu 22.04.4<br>(Jammy) |
| **ROS 2** | Humble Hawksbill<br>(EOL: 2027/5) | Humble Hawksbill<br>(EOL: 2027/5) |
| **Intel&reg; RealSense&trade;<br>Depth Camera** | Any D400-series | D456 |
| **LightWare LIDAR&copy;<br>Device** | SF40-C | SF45-B |

## Setup

First, follow the instructions in [SETUP.md](/SETUP.md) to prepare your development environment.

### *All-in-One*

To simplify setup, a [Terminator](https://gnometerminator.blogspot.com/p/introduction.html) layout is provided which prepares the project environment for you.

```bash
terminator -l ros2_layout &
```

- Terminal 1: sources ROS2 env, colcon builds, sources project env, clears window.
- Terminal 2: waits for colcon build, sources ROS2 and project envs, clears window.

> ***NOTE:*** this layout is defined in the file `~/.config/terminator/config`. If the layout is missing or you'd like to use it on your own machine, there is a backup provided in this repo at `tools/terminator/config`.

### *Manual*

First, build the `libra` package and its dependencies.

```bash
# Source a ROS2 workspace
. /opt/ros/humble/setup.bash
# Navigate to the workspace and build it
cd ros2_ws/
colcon build --packages-up-to libra
```

Then, source the project workspace and you're ready to go!

```bash
. install/local_setup.bash
```

> ***NOTE:*** for ROS 2 command tips, see [docs/ROS_Commands.md](/docs/ROS_COMMANDS.md).

## Usage

### *Launch Files*

Run one of the provided configurations.

- `libra_slam`: **primary launch file**; can launch a variety of RTAB-Map/sensor suite configurations.
- `libra_essentials`: nodes LIBRA App needs without SLAM (robot model TF, tip reach for gain switching); included by `libra_slam` whenever it runs kinematics.
- `model_only`: displays a model in RViz.
- `sensor_suite`: displays the sensor suite (RealSense + 2D LIDAR) model and data in RViz.
- `test_lidar`: displays 2D LIDAR data in RViz.
- `test_realsense`: displays RealSense data in RViz.

```bash
# Template
ros2 launch libra <configuration_name>.launch.py
```

For example, to run RTAB-Map in RGB+D mode with the RealSense as the main sensor and the 2D LIDAR as a corrective sensor, execute the following.

```bash
# Example
ros2 launch libra libra_slam.launch.py mode:=rgbd use_lidar:=true realsense_config:=/home/brice/repos/LIBRA-ROS/ros2_ws/install/libra/share/libra/config/realsense_all.yaml
```

> ***NOTE:*** each launch saves its own database, `rtabmap_<date>_<time>.db`, in `working_dir` and prints its path. It is recommended that you set `working_dir:=<PATH>` for each experiment. For example: `working_dir:=/home/brice/experiments/<YYYY-MM-DD>_<Title>`. To stop, press Ctrl+C **once** and wait for `Saving database/long-term memory...done!`.

#### **Archived**

The following have been superseded by `libra_slam` and are kept in `launch/archive`.

- `rtabmap_rgbd_2d-lidar`: launches RTAB-Map in RGB-D mode with laser scan correction, with the full sensor suite publishing data.
- `rtabmap_rgbd`: launches RTAB-Map in RGB-D mode with *only* RealSense publishing pre-packed RGB-D data.
- `rtabmap_stereo`: launches RTAB-Map in stereo mode with *only* RealSense publishing infrared data.

### *Individual Nodes*

#### **RealSense**

See [docs/REALSENSE.md](./docs/REALSENSE.md) for more details.

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -p pointcloud.enable:=true \
    -p enable_gyro:=true -p enable_accel:=true -p unite_imu_method:=2 \
    -p depth_module.depth_profile:=848x480x60 \
    -p depth_module.infra_profile:=848x480x60 \
    -p rgb_camera.color_profile:=848x480x60 \
    -p align_depth.enable:=true
```

#### **2D LIDAR**

See [docs/LIDAR.md](./docs/LIDAR.md) for more details.

```bash
ros2 run lightwarelidar2 sf45b --ros-args \
    -p port:=/dev/lidar -p frameId:=lidar_link -p updateRate:=12 \
    -p lowAngleLimit:=-160 -p highAngleLimit:=160
```

### *Recording a Session*

```bash
cd ros2_ws
mkdir bags && cd bags/
ros2 bag record <desired-topics>
# To stop recording, press Ctrl+C
```

Build a `ros2 bag record` command for your selected sensors by using the topic lists in the following subsections. Topics under ["Common"](#common) should always be captured. See [docs/ROS_COMMANDS.md "`record`"](./docs/ROS_COMMANDS.md#record) for more details.

When replaying data, you must launch the following nodes. See `libra_slam.launch.py` for relevant launch parameters.

- `robot_state_publisher`
- `rgbd_odometry` or `stereo_odometry`
- `rtabmap`
- `rtabmap_viz`

#### **Common**

```bash
/tf /tf_static /joint_states
```

> ***NOTE:*** Ideally, we would just record the /joint_states and let the `robot_state_publisher` generate the `/tf` and `/tf_static`, but since `realsense2_ros` also generates unique `/tf_static` for the camera, we have to record them.

#### **General Diagnostics**

Published by LIBRA App (see `libra_interfaces` for message definitions).

```bash
/joint_targets /libra/diagnostics /libra/cw_state /rosout
```

> ***NOTE:*** The `/libra/*` topics publish data from sources running at a variety of frequencies (for example: CW state: 30 Hz, fluid power: 4 Hz) at the same frequency as the fastest source (i.e., actuators: 100 Hz). Slower sources simply have their latest value republished.

#### **RGB+D Camera**

Published by `realsense2_camera` and `imu_filter_madgwick_node`.

```bash
/camera/color/image_raw /camera/color/camera_info /camera/aligned_depth_to_color/image_raw /imu/data
```

#### **Stereo Camera**

Published by `realsense2_camera` and `imu_filter_madgwick_node`.

```bash
/camera/infra1/camera_info /camera/infra1/image_rect_raw /camera/infra2/camera_info /camera/infra2/image_rect_raw /imu/data
```

#### **LIDAR**

Published by `sf45b`.

```bash
/pointcloud
```

## Troubleshooting

### *ROS 2 commands/nodes are having issues communicating*

If you followed `SETUP.md` and set your ROS 2 middleware to CycloneDDS, this may mean that your network environment is not playing nice with CycloneDDS. In that case, you can temporarily revert to using FastDDS with the following command.

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

> ***NOTE:*** packages like RTAB-Map, Nav2, and slam_toolbox require CycloneDDS to work.

### *RViz: "Wrong permissions on runtime directory"*

`QStandardPaths: wrong permissions on runtime directory /run/user/1000/ 0755 instead of 0700`.

If you get the above message whenever you open RViz in WSL2, just execute the following to persist the proper permissions.

```bash
chmod 0700 /run/user/1000/
```

### *RViz (or other Qt-based app) crashes on startup*

Your graphics drivers might be outdated or incompatible with the versions of the software used in this project. Particularly, **Qt-based GUI applications will suffer from frustrating crashes if your graphics drivers aren't up to date**. Follow the instructions below to add the appropriate [Mesa](https://www.mesa3d.org/) package repository (PPA) and upgrade your drivers.

1. Check your current Mesa/OpenGL version using `glxinfo`. Save the output in case you have to revert the update later.

    ```bash
    sudo apt install mesa-utils
    glxinfo | grep "OpenGL version"
    ```

2. Add the appropriate PPA for updating your Mesa/OpenGL drivers.
    > ***NOTE:*** If you're on Ubuntu 24.04 "Noble" you can pick either of the following PPAs (although the second one, `oibaf/graphics-drivers`, is recommended).
    - For Ubuntu 18.04 "Bionic" to 24.04 "Noble":

        ```bash
        sudo add-apt-repository ppa:kisak/kisak-mesa -y
        ```

    - For Ubuntu 24.04 "Noble" to 25.04 "Plucky":

        ```bash
        sudo add-apt-repository ppa:oibaf/graphics-drivers -y
        ```

3. Update your graphics libraries (note: this may also update other graphics-adjacent packages, such as OpenCV).

    ```bash
    sudo apt update && sudo apt upgrade
    ```

4. Check your updated Mesa/OpenGl version once again. If it shows a newer version, try running `rviz2` by itself now. On the other hand, if you are suddenly experiencing other/new graphical issues, revert to the previous version.

    ```bash
    glxinfo | grep "OpenGL version"
    ```

For reference, see [this guide](https://linuxcapable.com/how-to-upgrade-mesa-drivers-on-ubuntu-linux/) on LinuxCapable.com and [this discussion](https://www.reddit.com/r/ROS/comments/11wh0m3/wsl2_does_not_display_gazebo_and_rviz/) on the r/ROS subreddit.

### *RViz isn't loading the robot model*

If you see the following in RViz...

```bash
"No tf data. Actual error: Frame [base_link] does not exist"
```

... this may be a timing issue with RViz opening before `robot_state_publisher` finishes parsing the URDF (since it uses Xacro). Provided there are no errors in your terminal, just wait for a while and the model will eventually load.

## About Us

**Christian Brice** ([email](mailto:brice.c.67b9@m.isct.ac.jp)) is a doctoral student in mechanical engineering at the Institute of Science Tokyo (formerly: Tokyo Institute of Technology). The *LIBRA* project is the focus of his doctoral studies.

The **[Gen Endo Laboratory](www.robotics.mech.e.titech.ac.jp/gendo/en/)** is affiliated with the Department of Mechanical Engineering at the Institute of Science Tokyo.

## Contributing

1. Ensure Git is installed.

    ```bash
    sudo apt update
    sudo apt install git
    ```

    - If you're behind a proxy, you must also add your proxy settings to Git. In a terminal, input the following command (note: the `address` should be entered **with** the `http://` prefix).

        ```bash
        # (e.g., http://proxy.noc.titech.ac.jp:3128 for Gen Endo Lab)
        git config --global http.proxy <address>:<port>
        ```

2. Check out this project and all its submodules.

    ```bash
    git clone --recurse-submodules https://github.com/christian-brice/LIBRA-ROS.git
    ```
