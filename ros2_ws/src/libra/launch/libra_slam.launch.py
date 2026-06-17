# Brief:
#   Unified RTAB-Map launch file supporting multiple modes and odometry sources:
#   - Modes: RGB+D, Stereo (IR)
#   - Data: live (sensor nodes), replay (rosbag)
#   - Odometry: robot (actuator data), vio (visual-inertial data from RealSense)
#   - Additional Sensors: LIDAR
#
# Requirements:
#   An Intel RealSense D435i or D456
#   A LightWare SF45/B (for SLAM refinement only)
#   HEBI actuator states published on /joint_states (for robot odometry)
#   Install required ROS2 packages:
#     realsense2_camera (ros-$ROS_DISTRO-realsense2-camera)
#     lightwarelidar2 (lightwarelidar2)
#
# Usage:
#   Use defaults:
#     $ ros2 launch libra libra_slam.launch.py
#   RGB+D-only mode with all RealSense streams enabled and unique working directory:
#     $ ros2 launch libra libra_slam.launch.py \
#           slam_mode:=rgbd \
#           realsense_config:=/home/brice/repos/LIBRA-ROS/ros2_ws/install/libra/share/libra/config/realsense_all.yaml \
#           working_dir:=/home/brice/repos/LIBRA-ROS/ros2_ws/maps

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

# --- TABLE OF CONTENTS ---
#  !Launch Arguments
#  !Parameters and Remappings
#  !Mode-based Parameters
#  !Nodes
#

class c:  # ANSI color codes for terminal output
    RESET = '\033[0m'
    WARN = '\033[93m'  # yellow

def generate_launch_description():
    # Get general paths
    default_working_dir = os.path.join(os.path.expanduser('~'), '.rtabmap')
    
    # Get package-specific paths
    pkg_share = get_package_share_directory('libra')
    config_dir = os.path.join(pkg_share, 'config')
    default_urdf_model_path = os.path.join(pkg_share, 'models', 'robots', 'libra1.urdf.xacro')

    #--------------------------------------------------------------------------
    # !Launch Arguments
    #--------------------------------------------------------------------------

    declare_script_mode_cmd = DeclareLaunchArgument(
        'script_mode',
        default_value='full_stack',
        choices=['full_stack', 'record', 'replay', 'replay_w_kinematics', 'kinematics_only'],
        description='Script mode: "full_stack" launches the entire LIBRA SLAM pipeline | "record" only launches sensors and related nodes | "replay" and "replay_w_kinematics" only launch SLAM nodes (reliant on rosbag playback) | "kinematics_only" only launches kinematic nodes (reliant on rosbag playback).'
    )

    declare_slam_mode_cmd = DeclareLaunchArgument(
        'slam_mode',
        default_value='rgbd',
        choices=['rgbd', 'stereo'],
        description='RTAB-Map mode: "rgbd" for RGB + depth data, "stereo" for stereoscopic infrared data.'
    )

    declare_odom_source_cmd = DeclareLaunchArgument(
        'odom_source',
        default_value='robot',
        choices=['robot', 'vio'],
        description='Odometry source: "robot" for EKF-fused actuator + IMU data, "vio" for visual-inertial odometry fusion.'
    )

    declare_urdf_model_cmd = DeclareLaunchArgument(
        'urdf_model',
        default_value=default_urdf_model_path,
        description='Path to URDF file with robot model definition.'
    )

    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id',
        default_value='',  # set conditionally based on slam_mode
        description='Robot model base frame ID. If empty, value determined by slam_mode.'
    )

    declare_realsense_config_cmd = DeclareLaunchArgument(
        'realsense_config',
        default_value='',  # set conditionally based on slam_mode
        description='Path to YAML file with RealSense camera parameters. If empty, value determined by slam_mode.'
    )
    
    declare_use_lidar_cmd = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Whether to use 2D LIDAR data to enhance SLAM accuracy.'
    )

    declare_lidar_port_cmd = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/lidar',
        description='Serial port for 2D LIDAR. (use_lidar:=true only)'
    )

    declare_working_dir_cmd = DeclareLaunchArgument(
        'working_dir',
        default_value=default_working_dir,
        description='Path to RTAB-Map working directory. Defaults to ".rtabmap/" in your user home.'
    )

    declare_delete_db_cmd = DeclareLaunchArgument(
        'delete_db',
        default_value='false',
        description='Delete previous RTAB-Map database on launch.'
    )

    #--------------------------------------------------------------------------
    # !Parameters and Remappings
    #--------------------------------------------------------------------------

    # NOTE: For more detailed information on RTAB-Map parameters, see
    #       https://github.com/introlab/rtabmap/wiki/Change-parameters
    #         and
    #       https://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning
    
    # NOTE: Parameters that are lowercase and use underscore separators take
    #       Pythonic values (e.g., 30 or True). All others take string values
    #       (e.g., '30' or 'True').

    # NOTE: Apparently, some parameters that are part of the "RGBD" category are
    #       not actually exclusive to RGBD-based odometry.

    # Common
    
    base_rtab_params = {
        # General
        'sync_queue_size': 30,  # default: 10
        'topic_queue_size': 30,  # default: 10
        'qos': 2,  # 0 = SYSTEM_DEFAULT, 1 = RELIABLE, 2 = BEST_EFFORT
        'wait_imu_to_init': True,
        # Mapping
        'RGBD/OptimizeFromGraphEnd': 'false',  # correct all previous poses based on latest pose (/odom and /map will always match)
        'RGBD/OptimizeStrategy': '2',  # 1 = g2o, 2 = GTSAM
        'RGBD/OptimizeRobust': 'true',  # use Vertigo to reduce the influence of "weak" (outlier) loop closures
        'RGBD/OptimizeMaxError': '0',  # must be 0 if RGBD/OptimizeRobust is true
        # Odometry and Registration
        'Kp/NndrRatio': '0.80',  # default: 0.80
        'Kp/NNStrategy': '1',  # FLANN KdTree, build for float descriptors like SIFT
        'Odom/FillInfoData': 'true',
        'Odom/MinInliers': '20',
        'Reg/Force3DoF': 'true',  # roll, pitch, and Z must be explicitly provided (i.e., won't be estimated)
        'Vis/FeatureType': '1',  # SIFT, better than GFTT+ORB in large-scale env
        'Vis/MaxDepth': '6.0',  # RealSense D456's max accurate range
    }

    # Mode-specific
    
    rgbd_params = base_rtab_params | {
        # Input Topics
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_rgbd': False,  # let RTAB-Map take care of combining them
        # Mapping
        'RGBD/AngularUpdate': '0.02',  # m
        'RGBD/LinearUpdate': '0.02',  # rad
    }
    rgbd_remaps = [
        # Inputs: realsense2_camera
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        # Inputs: imu_filter_madgwick_node
        ('imu', '/imu/data'),
    ]

    stereo_params = base_rtab_params | {
        # Input Topics
        'subscribe_stereo': True,
        # Mapping
        'RGBD/AngularUpdate': '0.05',  # m, higher than rgbd_params 
        'RGBD/LinearUpdate': '0.05',  # rad, 
    }
    stereo_remaps = [
        # Inputs: realsense2_camera
        ('left/image_rect', '/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/infra2/camera_info'),
        # Inputs: imu_filter_madgwick_node
        ('imu', '/imu/data'),
    ]

    # Odom-specific

    robot_odom_params = {
        # General
        'subscribe_odom_info': False,  # OdomInfo is only published by RTAB-Map odom nodes
        # Odometry and Registration
        'Odom/Strategy': '0',  # rely on external odometry (F2M, or Frame-to-Map)
    }
    robot_odom_remaps = [
        # Inputs: kinematic_odometry_publisher.py
        ('odom', '/robot_odom'),
    ]

    vio_params = {
        # General
        'subscribe_odom_info': True,  # published by rgbd_odometry or stereo_odometry
    }

    # Sensor-specific

    lidar_params = {
        # Input Topics
        'subscribe_scan_cloud': True,
        # Mapping
        'Grid/FromDepth': 'false',  # true: depth, false: laser scans
        'Grid/Sensor': '1',  # construct occupancy grid map using cloud projection
        'RGBD/NeighborLinkRefining': 'true',  # correct odom with laser scans + ICP
        'RGBD/ProximityBySpace': 'true',  # find local loop closures based on robot position using ICP
        # Odometry and Registration
        'Reg/Strategy': '2',  # improve VSLAM with laser scans
        'Icp/VoxelSize': '0.05',  # in m; filter scans down to 5 cm voxel before registration
        'Icp/MaxCorrespondenceDistance': '0.1',  # in m; max distance between points during registration
    }
    lidar_remaps = [
        # Inputs: sf45b
        ('scan_cloud', '/pointcloud'),
    ]

    def launch_nodes(context, *args, **kwargs):
        """
        Generates a list of nodes to be launched based launch arguments.
        This is supplied to the ROS2 `LaunchDescription` via `OpaqueFunction`.

        Usage:
            OpaqueFunction(function=launch_nodes)

        Args:
            context: Context object containing the launch configurations.
            *args: Additional arguments (unused).
            **kwargs: Additional keyword arguments (unused).
        """
        
        # Get constant parameters
        script_mode = context.launch_configurations['script_mode']
        slam_mode = context.launch_configurations['slam_mode']
        odom_source = context.launch_configurations['odom_source']
        urdf_model = context.launch_configurations['urdf_model']
        use_lidar = context.launch_configurations['use_lidar'].lower() == 'true'
        lidar_port = context.launch_configurations['lidar_port']
        working_dir = context.launch_configurations['working_dir']
        delete_db = context.launch_configurations['delete_db'].lower() == 'true'

        # Get variable parameters
        frame_id_arg = context.launch_configurations['frame_id']
        realsense_config_arg = context.launch_configurations['realsense_config']

        #----------------------------------------------------------------------
        # !Conditional Parameters
        #----------------------------------------------------------------------

        rtab_params = {}
        remaps = []
        odom_extra = {}
        slam_extra = {}

        # Build individual node parameters based on launch arguments
        if slam_mode == 'rgbd':
            realsense_config = realsense_config_arg or os.path.join(config_dir, 'realsense_rgbd.yaml')
            rtab_params |= rgbd_params
            remaps += rgbd_remaps
            odom_exec = 'rgbd_odometry'
        else:  # slam_mode == 'stereo'
            realsense_config = realsense_config_arg or os.path.join(config_dir, 'realsense_stereo.yaml')
            rtab_params |= stereo_params
            remaps += stereo_remaps
            odom_exec = 'stereo_odometry'

        if odom_source == 'robot':
            rtab_params |= robot_odom_params
            remaps += robot_odom_remaps
        else:  # odom_source == 'vio'
            rtab_params |= vio_params
            odom_extra |= {'Reg/Strategy': '0'}  # VIO inherently can't use laser scans

        if use_lidar:
            frame_id = frame_id_arg or 'sensor_suite_base_link'  # nearest common link to RealSense and LIDAR
            rtab_params |= lidar_params
            remaps += lidar_remaps
        else:  # !use_lidar
            frame_id = frame_id_arg or 'camera_link'
        
        # Finish modifying common parameters based on variables set above
        rtab_params |= {'frame_id': frame_id}

        # TODO: the following is for testing purposes only
        slam_extra |= {
            'Rtabmap/LoopThr': '0.05',  # temporarily lower from 0.11
            'RGBD/AngularUpdate': '0.1',  # temporarily raise from 0.01  
            'RGBD/LinearUpdate': '0.1',   # temporarily raise from 0.01
        }

        #----------------------------------------------------------------------
        # !Nodes
        #----------------------------------------------------------------------

        kinematics_en = False
        camera_en = False
        lidar_en = False
        rtabmap_en = False

        # Determine which nodes will run
        if script_mode == 'full_stack':
            kinematics_en = True
            camera_en = True
            if use_lidar:
                lidar_en = True
            rtabmap_en = True

        elif script_mode == 'record':
            kinematics_en = True
            camera_en = True
            if use_lidar:
                lidar_en = True

        elif 'replay' in script_mode:  # 'replay' or 'replay_w_kinematics'
            rtabmap_en = True
            if script_mode == 'replay_w_kinematics':
                kinematics_en = True

            print(f'{c.WARN}[WARN] In replay mode, no input nodes are instantiated.\n  Please run `ros2 bag play <filename>` in a separate terminal when you\'re ready.{c.RESET}')
            time.sleep(2)  # give user time to read warning

        elif 'kinematics_only' in script_mode:
            kinematics_en = True

            print(f'{c.WARN}[WARN] In kinematics_only mode, no input nodes are instantiated.\n  Please run `ros2 bag play <filename>` in a separate terminal when you\'re ready.{c.RESET}')
            time.sleep(2)  # give user time to read warning

        else:
            raise ValueError(f"Invalid script_mode: {script_mode}")

        nodes = []

        # --- Kinematics & Odometry Nodes ---

        if kinematics_en:
            # Publish robot frame transforms based on URDF model
            nodes.append(Node(
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
                        Command(['xacro ', urdf_model]), value_type=str
                    )
                }]
            ))
        
            # --- Conditional Odometry Nodes ---

            if odom_source == 'robot':
                # Publish static transform from 'odom' -> 'base_link'
                nodes.append(Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='odom_to_base_link_publisher',
                    arguments=[
                        '0', '0', '0',  # no X, Y, Z translation
                        '0', '0', '0',  # no yaw, pitch, roll rotation
                        'odom', 'base_link'  # parent and child frames
                    ]
                ))

                # Kinematic odometry publisher - computes sensor suite pose from joint states
                nodes.append(Node(
                    package='libra',
                    executable='kinematic_odometry_publisher.py',
                    output='screen',
                    parameters=[{
                        'child_frame': frame_id,
                        'publish_frequency': 50.0
                    }]
                ))

                # NOTE: The following doesn't work but I'm keeping it for reference.
                #       Specifically, RTAB-Map needs a non-static /odom in order to
                #       attempt a loop closure. Even if /tf isn't static, and the
                #       sensors move relative to the base_link, SLAM will not work.
                #
                # Publish static odometry message to RTAB-Map
                #nodes.append(Node(
                #    package='libra',
                #    executable='static_odometry_publisher.py',
                #    output='screen',
                #))
            
            elif odom_source == 'vio' and rtabmap_en:
                # Compute visual-inertial odometry
                nodes.append(Node(
                    package='rtabmap_odom',
                    executable=odom_exec,
                    output='screen',
                    parameters=[ rtab_params | odom_extra | {
                        'Rtabmap/WorkingDirectory': working_dir
                    }],
                    remappings=remaps
                ))

        # --- Sensor Nodes ---

        if camera_en:
            # RealSense camera
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    )
                ]),
                launch_arguments={
                    'camera_namespace': '',  # avoid nesting "/camera/camera"
                    'config_file': realsense_config
                }.items()
            ))

            # Compute quaternion of the IMU
            nodes.append(Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                output='screen',
                parameters=[{
                    'use_mag': False,
                    'world_frame': 'enu',
                    'publish_tf': False
                }],
                remappings=[
                    # Inputs: realsense2_camera
                    ('imu/data_raw', '/camera/imu')
                ]
            ))

        if lidar_en:
            # LightWare 2D LIDAR
            nodes.append(Node(
                package='lightwarelidar2',
                executable='sf45b',
                output='screen',
                parameters=[{
                    'port': lidar_port,
                    'frameId': 'lidar_link',
                    'baudrate': 115200,
                    'updateRate': 12,  # fastest rotation rate  
                    'lowAngleLimit': -160,  # max scan angles
                    'highAngleLimit': 160
                }],
            ))

        # --- SLAM Nodes ---

        if rtabmap_en:
            # Primary RTAB-Map SLAM computation
            nodes.append(Node(
                package='rtabmap_slam',
                executable='rtabmap',
                output='screen',
                parameters=[ rtab_params | slam_extra | {
                    'Rtabmap/WorkingDirectory': working_dir
                }],
                remappings=remaps,
                arguments=['-d'] if delete_db else []
            ))

            # RTAB-Map's custom RViz GUI
            nodes.append(Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                output='screen',
                parameters=[ rtab_params ],
                remappings=remaps
            ))
        
        return nodes

    return LaunchDescription([
        # Launch arguments
        declare_script_mode_cmd,
        declare_slam_mode_cmd,
        declare_odom_source_cmd,
        declare_urdf_model_cmd,
        declare_frame_id_cmd,
        declare_realsense_config_cmd,
        declare_use_lidar_cmd,
        declare_lidar_port_cmd,
        declare_working_dir_cmd,
        declare_delete_db_cmd,
        # Nodes
        OpaqueFunction(function=launch_nodes)
    ])
