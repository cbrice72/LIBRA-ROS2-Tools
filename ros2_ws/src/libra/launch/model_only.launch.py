# Brief:
#   Debugger launch file for visualizing URDF files:
#   - arm: the 2-DoF Joint and Arm (i.e., everything but the Counterweight)
#   - sensor_suite: the RealSense, 2D LIDAR, and their mounts
#
# Requirements:
#   (none)
#
# Usage:
#   $ ros2 launch libra model_only.launch.py urdf_model:=arm

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

# --- TABLE OF CONTENTS ---
#  !Launch Arguments
#  !Nodes
#

def generate_launch_description():
    # Get package-specific paths
    pkg_share = get_package_share_directory('libra')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'rviz_basic_settings.rviz')

    libra1_urdf = os.path.join(pkg_share, 'models', 'robots', 'libra1.urdf.xacro')
    libra1_no_manip_urdf = os.path.join(pkg_share, 'models', 'robots', 'libra1_no_manip.urdf.xacro')
    sensor_suite_urdf = os.path.join(pkg_share, 'models', 'display_sensors.urdf.xacro')

    # --------------------------------------------------------------------------
    # !Launch Arguments
    # --------------------------------------------------------------------------

    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_model_arg = DeclareLaunchArgument(
        name='urdf_model',
        default_value='libra1',
        description='Model to load: "libra1", "libra1_basic", or "sensor_suite" (default: libra1)')

    rviz_config_file_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to RViz config file (default: rviz_basic_settings.rviz)')

    use_joint_state_pub_arg = DeclareLaunchArgument(
        name='use_joint_state_pub',
        default_value='True',
        description='Whether to start the joint_state_publisher node')

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RViz')

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Whether to use simulation (Gazebo) clock')

    # Resolve URDF path with strict checking
    urdf_path_expr = PythonExpression([
        '"', libra1_urdf, '" if "', urdf_model, '" == "libra1" else ',
        '("', libra1_no_manip_urdf, '" if "', urdf_model, '" == "libra1_no_manip" else ',
        '("', sensor_suite_urdf, '" if "', urdf_model, '" == "sensor_suite" else ',
        'exit("Invalid urdf_model value. Must be \'libra1\', \'libra1_no_manip\', or \'sensor_suite\'")))'
    ])

    #----------------------------------------------------------------------
    # !Nodes
    #----------------------------------------------------------------------

    # Publish joint state values for non-fixed joints (see URDF)
    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # Subscribe to joint states of robot and publish 3D pose of each link
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'publish_frequency': 10.0,
            'use_sim_time': use_sim_time,
            # Note: If ANY text in the xacro output can be interpreted as yaml,
            # the roslaunch system will try to interpret the ENTIRE text as yaml
            # instead of passing on the string. The biggest cause of this false
            # interpretation is commenting out xacro calls since the xacro:property
            # or similar looks a lot like a yaml key:value pair. Comments are not
            # removed by xacro so they are included in the output.
            # (source: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/)
            'robot_description': ParameterValue(Command(['xacro ', urdf_path_expr]), value_type=str)
        }]
    )

    # Launch RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    return LaunchDescription([
        # Launch arguments
        urdf_model_arg,
        rviz_config_file_arg,
        use_joint_state_pub_arg,
        use_rviz_arg,
        use_sim_time_arg,
        # Nodes
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
