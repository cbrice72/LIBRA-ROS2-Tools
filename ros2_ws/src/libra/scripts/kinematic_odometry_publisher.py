#!/usr/bin/env python3

# Brief:
#   Publishes odometry based on existing TF data from robot_state_publisher.
#   Computes velocities using real feedback values from joint states.
#
# Usage:
#   $ ros2 run libra kinematic_odometry_publisher

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np

class KinematicOdometryPublisher(Node):
    """
    Minimal odometry publisher using TF lookups from an active robot_state_publisher.
    """
    
    def __init__(self):
        super().__init__('kinematic_odometry_publisher_node')

        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('target_frame', 'sensor_suite_base_link')  # TODO: check if this is supposed to be base_link
        self.declare_parameter('publish_frequency', 50.0)

        self.odom_frame_ = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.target_frame_ = self.get_parameter('target_frame').get_parameter_value().string_value
        self.publish_frequency_ = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # TF setup
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        
        # Joint state tracking (merged by name, since each node publishes its own joints)
        self.joint_velocities_ = {}
        self.expected_joints_ = ['Roll', 'Pitch', 'J1', 'J2', 'J3']  # Static-Manip is always 0
        
        # Publishers and Subscribers
        self.joint_sub_ = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, '/robot_odom', 10)
        
        # Timer
        timer_period = 1.0 / self.publish_frequency_
        self.timer_ = self.create_timer(timer_period, self.publish_odometry)

        self.get_logger().info(
            f"Kinematic Odometry Publisher started.\n"
            f"  Publishing TF '{self.odom_frame_} -> {self.target_frame_}' as odometry at {self.publish_frequency_} Hz\n"
            f"  Using joint velocities from /joint_states for twist calculation"
        )

    def joint_states_callback(self, msg):
        """
        Store the latest velocity of each joint in the message, for twist computation.
        (Mocked joints carry no velocity, so they're skipped.)
        """
        self.joint_velocities_.update(zip(msg.name, msg.velocity))

    def publish_odometry(self):
        """
        Look up current pose from TF and publish as odometry.
        """
        try:
            # Get current transform
            now = rclpy.time.Time()
            t = self.tf_buffer_.lookup_transform(self.odom_frame_, self.target_frame_, now)
            
            current_time = self.get_clock().now()
            
            # Create odometry message and prepare header
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = self.odom_frame_
            odom_msg.child_frame_id = self.target_frame_
            
            # Set pose from TF
            odom_msg.pose.pose.position.x = t.transform.translation.x
            odom_msg.pose.pose.position.y = t.transform.translation.y
            odom_msg.pose.pose.position.z = t.transform.translation.z
            odom_msg.pose.pose.orientation.x = t.transform.rotation.x
            odom_msg.pose.pose.orientation.y = t.transform.rotation.y
            odom_msg.pose.pose.orientation.z = t.transform.rotation.z
            odom_msg.pose.pose.orientation.w = t.transform.rotation.w
            
            # Compute velocity using joint velocities (if available)
            twist = self.compute_velocity_from_joints()
            if twist is not None:
                odom_msg.twist.twist.linear.x = twist[0]
                odom_msg.twist.twist.linear.y = twist[1]
                odom_msg.twist.twist.linear.z = twist[2]
                odom_msg.twist.twist.angular.x = twist[3]
                odom_msg.twist.twist.angular.y = twist[4]
                odom_msg.twist.twist.angular.z = twist[5]
            else:
                # Fallback to zeros
                odom_msg.twist.twist.linear.x = 0.0
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0
                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = 0.0
            
            # Set reasonable covariance (low values = high confidence)
            # NOTE: Variances go on the diagonal only. A 6x6 matrix filled with one value
            #       can't be inverted, which makes RTAB-Map abort on its first node.
            pose_cov = np.diag([0.001] * 6).ravel().tolist()
            twist_cov = np.diag([0.01] * 6).ravel().tolist()  # slightly less confident in velocities
            odom_msg.pose.covariance = pose_cov
            odom_msg.twist.covariance = twist_cov
            
            # Publish
            self.odom_pub_.publish(odom_msg)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=1.0)

    def compute_velocity_from_joints(self):
        """
        Compute end-effector velocities from joint velocities.
        This is a simplified approximation - for exact velocities, compute the Jacobian matrix.
        Returns [vx, vy, vz, wx, wy, wz] or None if no joint data available.
        """
        missing = [j for j in self.expected_joints_ if j not in self.joint_velocities_]
        if missing:
            self.get_logger().warn(f"No velocity yet for joints: {missing}", throttle_duration_sec=5.0)
            return None

        # Extract joint velocities
        roll_vel, pitch_vel, j1_vel, j2_vel, j3_vel = (
            self.joint_velocities_[j] for j in self.expected_joints_)
        
        # Over-simplified approximation
        # TODO: Compute 6x6 Jacobian matrix `J` and use `v = J * q_dot`

        # Linear velocities
        arm_reach = 4.0  # approximate total arm reach in meters
        linear_vel = [
            arm_reach * (j1_vel + j2_vel) * 0.5,  # vx 
            arm_reach * (j1_vel + j2_vel) * 0.3,  # vy
            arm_reach * pitch_vel * 0.2           # vz
        ]
        
        # Angular velocities (direct from joint rates)
        angular_vel = [
            roll_vel,            # vroll (about x)
            pitch_vel + j3_vel,  # vpitch (about y)  
            j1_vel + j2_vel      # vyaw (about z)
        ]
        
        return linear_vel + angular_vel


def main(args=None):
    rclpy.init(args=args)
    node = KinematicOdometryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
