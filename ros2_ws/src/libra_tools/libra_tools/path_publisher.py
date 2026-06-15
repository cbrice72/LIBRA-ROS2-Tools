# Brief:
#   A simple ROS2 node that publishes a persistent Path message for a given TF
#   frame. Can be used to, e.g., visualize the tip path of a robotic arm in RViz.
#
# Usage:
#   ros2 run libra_tools path_publisher
#
# Changing Parameters at Runtime:
#   You can change base_frame_id and child_frame_id at runtime via the ROS2
#   parameter services. Note that changing these parameters will reset the
#   current path history.
#     ros2 param set /tip_path_publisher base_frame_id "map"
#     ros2 param set /tip_path_publisher child_frame_id "camera_link"

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration

from rcl_interfaces.msg import SetParametersResult

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


# The fixed frame to measure from (usually 'map' or 'odom')
BASE_FRAME_ID = 'base_link'
# The frame to track (i.e., the tip)
CHILD_FRAME_ID = 'manip_out_link'
# Published Path topic (to be consumed by RViz)
PATH_TOPIC = '/tip_path'
# Frequency (in seconds) for checking the transform/publishing the path
PUB_FREQ = 0.1  # 10 Hz


class PathPublisher(Node):
    """
    Publishes a persistent nav_msgs/Path message for the given frame.
    """

    def __init__(self):
        super().__init__('tip_path_publisher')

        # Parameters
        self.declare_parameter('base_frame_id', BASE_FRAME_ID)
        self.declare_parameter('child_frame_id', CHILD_FRAME_ID)

        self._base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self._child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self.get_logger().info(f"Starting Path Publisher for transform: {self._base_frame_id} -> {self._child_frame_id}")

        # Enable runtime parameter changes
        self.add_on_set_parameters_callback(self._on_param_change)

        # Initialize TF buffer and listener
        self._tf_buffer = tf2_ros.Buffer(Duration(seconds=10))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Initialize Path publisher and message
        self._path_pub = self.create_publisher(Path, PATH_TOPIC, 10)
        self._path_msg = Path()
        self._path_msg.header.frame_id = self._base_frame_id
        
        # Create timer for update loop
        self._timer = self.create_timer(PUB_FREQ, self._update_path)


    def _update_path(self):
        """
        Main loop that looks up the transform, updates the path history, and
        publishes the result.
        """
        # Update timestamp
        self._path_msg.header.stamp = self.get_clock().now().to_msg()

        # Check if necessary data is being published
        if not self._tf_buffer.can_transform(self._base_frame_id, self._child_frame_id, Time()):
            self.get_logger().warn(
                f"Data not available: '{self._base_frame_id}' and/or '{self._child_frame_id}' do not exist in the TF tree yet", 
                throttle_duration_sec=5.0
            )
            return
        
        try:
            # Look up TF (Fixed Frame -> Child Frame)
            transform = self._tf_buffer.lookup_transform(
                self._base_frame_id,
                self._child_frame_id,
                Time(), # use latest
                Duration(seconds=1.0) # timeout
            )

            # Convert to PoseStamped
            current_pose = PoseStamped()
            current_pose.header = transform.header
            current_pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.orientation = transform.transform.rotation

            # Append current pose to Path and publish
            self._path_msg.poses.append(current_pose)
            self._path_pub.publish(self._path_msg)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Handle errors without failing
            self.get_logger().warn(f"TF failure looking up {self._base_frame_id} to {self._child_frame_id}: {e}", throttle_duration_sec=1.0)

    def _on_param_change(self, params):
        """
        Callback for handling runtime parameter changes.
        """
        for param in params:
            if param.name == 'base_frame_id' and param.type_ == rclpy.Parameter.Type.STRING:
                self._base_frame_id = param.value
                self._path_msg.header.frame_id = self._base_frame_id
                self._path_msg.poses.clear()  # clear existing path

                self.get_logger().info(f"Updated base frame ID to: {self._base_frame_id}")

            elif param.name == 'child_frame_id' and param.type_ == rclpy.Parameter.Type.STRING:
                self._child_frame_id = param.value
                self._path_msg.poses.clear()  # clear existing path

                self.get_logger().info(f"Updated child frame ID to: {self._child_frame_id}")

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
