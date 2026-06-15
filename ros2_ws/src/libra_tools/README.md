# LIBRA Tools

Supplemental ROS2 tools for the LIBRA Project to track, record, and analyze specific data. Prepare your environment by running the following commands in the `ros2_ws` directory.

```bash
colcon build --packages-select libra_tools
. install/setup.bash
```

- [Tip Path Utilities](#tip-path-utilities)
    - [*`libra_tools/path_publisher.py`*](#libra_toolspath_publisherpy)
    - [*`libra_tools/path_to_csv.py`*](#libra_toolspath_to_csvpy)
    - [*`scripts/camera_path_analyzer.py`*](#scriptscamera_path_analyzerpy)

## Tip Path Utilities

### *`libra_tools/path_publisher.py`*

ROS2 node that publishes **persistent** path messages (`nav_msgs/Path`) by tracking the TF between the RealSense camera link and the base.

Usage:

```bash
ros2 run libra_tools path_publisher
# You can also set params during runtime
ros2 param set /tip_path_publisher base_frame_id "map"
ros2 param set /tip_path_publisher child_frame_id "camera_link"
```

### *`libra_tools/path_to_csv.py`*

ROS2 node that reads the entire history of path messages (`nav_msgs/Path`) of the default ROS2 domain and saves the timestampted poses to `tip_path.csv`.

> ***NOTE:*** Since this node reads all persistent path messages at once, it should be run **at the end** of an experiment or rosbag.

Usage:

```bash
ros2 run libra_tools path_to_csv
```

### *`scripts/camera_path_analyzer.py`*

Helper script that reads a `.csv` file with timestamped poses and models a RealSense D456's resulting FOV using Monte-Carlo ray-casting.

Usage:

```bash
python3 camera_path_analyzer.py
```
