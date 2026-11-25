#!/usr/bin/env bash

# Records a predetermined group of topics to a rosbag.
#   "full":  everything needed to replay SLAM and rebuild the online map
#   "light": joint data, counterweight, IMU (camera), and logs only
#
# Usage:
#   ./record_run.bash <experiment_dir> <full|light>
#   (press Ctrl+C to stop recording)
#
# Notes:
#   For "full", don't forget to launch libra_slam.launch.py with
#   `working_dir:=<experiment_dir>`.
#
# TODO: check Claude-generated code

# ==================== LOCAL VARIABLES AND FUNCTIONS ====================

# Recorded every run
common_topics=(
    /tf
    /tf_static
    /joint_states
    /joint_targets
    /libra/diagnostics
    /libra/cw_state
    /rosout
    /imu/data    # filtered IMU w/ orientation
    /camera/imu  # raw IMU at full rate
)

# "Full" runs only
full_topics=(
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/aligned_depth_to_color/image_raw
    /pointcloud  # 2D LIDAR
    /odom        # visual-inertial odometry
    /mapGraph    # optimized map poses
)

color_echo() {
    local level=$1
    shift
    local msg="$*"

    case "$level" in
        INFO)
            echo -e "\033[1;36m[INFO]\033[0;36m $msg\033[0m"
            ;;
        WARN)
            echo -e "\033[1;33m[WARN]\033[0;33m $msg\033[0m"
            ;;
        ERROR)
            echo -e "\033[1;31m[ERROR]\033[0;31m $msg\033[0m"
            ;;
        *)
            echo "[UNKNOWN] $msg"
            ;;
    esac
}

# ==================== SCRIPT START ====================

# Check arguments
if [[ "$#" -ne 2 || ! "$2" =~ ^(full|light)$ ]]; then
    color_echo ERROR "Usage: $0 <experiment_dir> <full|light>"
    exit 1
fi
experiment_dir=$1
profile=$2

topics=("${common_topics[@]}")
if [[ "$profile" == "full" ]]; then
    topics+=("${full_topics[@]}")
fi

# Warn about topics that aren't published yet
# (rosbag2 will record them if they start later)
published=$(ros2 topic list --no-daemon --spin-time 2 2>/dev/null)
for topic in "${topics[@]}"; do
    if ! grep -qx -- "$topic" <<< "$published"; then
        color_echo WARN "Not published yet: $topic"
    fi
done

# Record into a unique rosbag (timestamped)
mkdir -p "$experiment_dir/log" || exit 1
bag_dir="$experiment_dir/log/rosbag2_$(date +%Y-%m-%d_%H-%M-%S)_$profile"
color_echo INFO "Recording ${#topics[@]} topics ($profile) to: $bag_dir"
color_echo INFO "Press Ctrl+C once to stop."

ros2 bag record -o "$bag_dir" "${topics[@]}"
