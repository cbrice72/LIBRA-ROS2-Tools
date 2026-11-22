# Brief:
#   Nodes LIBRA App needs on the ROS2 side, regardless of whether SLAM is used:
#   - robot_state_publisher: TF from the URDF model
#   - tip_reach_server: tip reach of requested poses (for automatic gain scheduling)
#
# Usage:
#   $ ros2 launch libra libra_essentials.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

# --- TABLE OF CONTENTS ---
#  !Launch Arguments
#  !Nodes
#

def generate_launch_description():
    # Get package-specific paths
    pkg_share = get_package_share_directory('libra')
    default_urdf_model_path = os.path.join(pkg_share, 'models', 'robots', 'libra1.urdf.xacro')

    #--------------------------------------------------------------------------
    # !Launch Arguments
    #--------------------------------------------------------------------------

    declare_urdf_model_cmd = DeclareLaunchArgument(
        'urdf_model',
        default_value=default_urdf_model_path,
        description='Path to URDF file with robot model definition.'
    )

    #--------------------------------------------------------------------------
    # !Nodes
    #--------------------------------------------------------------------------

    # Publish robot frame transforms based on URDF model
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            # NOTE: If ANY text in the xacro output can be interpreted as yaml,
            # the roslaunch system will try to interpret the ENTIRE text as yaml
            # instead of passing on the string. The biggest cause of this false
            # interpretation is commenting out xacro calls since the xacro:property
            # convention looks a lot like a yaml key:value pair. Comments are not
            # removed by xacro so they are included in the output.
            # (source: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/)
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('urdf_model')]), value_type=str
            )
        }]
    )

    # Tip reach of requested poses, for LIBRA App's gain scheduling
    tip_reach_server_node = Node(
        package='libra',
        executable='tip_reach_server.py',
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        declare_urdf_model_cmd,
        # Nodes
        robot_state_publisher_node,
        tip_reach_server_node
    ])
