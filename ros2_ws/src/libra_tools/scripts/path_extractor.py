#!/usr/bin/env python3

# Brief:
#   A single-use ROS2 node that extracts a 3D trajectory from a rosbag's TF data
#   and saves it to a CSV file.
#
# Usage:
#   python3 path_extractor.py /path/to/bag_dir
#
# Optional Args:
#   --base BASE_FRAME_ID (default: 'base_link')
#   --child CHILD_FRAME_ID (default: 'manip_out_link')
#   --out OUT_FILENAME (default: 'tip_path')
#   --freq PUB_FREQ (default: 0.1 seconds, i.e., 10 Hz sampling)

import argparse
import csv
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.time import Time, Duration
from rosidl_runtime_py.utilities import get_message

import tf2_ros
from tf2_ros import TransformException
import rosbag2_py

# Defaults
BASE_FRAME_ID = 'base_link'
CHILD_FRAME_ID = 'manip_out_link'
OUT_FILENAME = 'tip_path'
# Frequency (in seconds) for sampling the transform
PUB_FREQ = 0.1  # 10 Hz


class PathExtractor(Node):
    """
    Parses a rosbag, listens to TF messages, and writes the poses to a CSV.
    """

    def __init__(self, bag_path, base_frame, child_frame, out_filename, freq):
        super().__init__('path_extractor')

        self._bag_path = bag_path
        self._base_frame_id = base_frame
        self._child_frame_id = child_frame
        self._out_filepath = os.path.join(os.getcwd(), out_filename + '.csv')
        self._sample_period_ns = int(freq * 1e9)

        self._path_data = []

        self._tf_buffer = tf2_ros.Buffer(
            Duration(seconds=36000.0),  # huge buffer, just in case
            node=self)
        self._tf_msg_type = get_message('tf2_msgs/msg/TFMessage')

    def _open_bag_reader(self):
        """
        Initializes and returns a rosbag2 reader filtered for TF topics.
        """
        storage_options = rosbag2_py.StorageOptions(uri=self._bag_path, storage_id='')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(storage_options, converter_options)
        except Exception as e:
            self.get_logger().error(f"Failed to open bag: {e}")
            sys.exit(1)

        storage_filter = rosbag2_py.StorageFilter(topics=['/tf', '/tf_static'])
        reader.set_filter(storage_filter)
        
        return reader

    def _save_to_csv(self):
        """
        Writes processed path data to a CSV file.
        """
        self.get_logger().info(f"Extracted {len(self._path_data)} poses")

        with open(self._out_filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'stamp_sec'])
            writer.writerows(self._path_data)

        self.get_logger().info(f"File saved to: {self._out_filepath}")

    def extract(self):
        """
        Reads TFs and their timestamps from the rosbag and saves them to a CSV file.
        """
        self.get_logger().info(f"Starting extraction from: {self._bag_path}")
        self.get_logger().info(f"Tracking transform: '{self._base_frame_id}' -> '{self._child_frame_id}'")

        reader = self._open_bag_reader()

        self.get_logger().info("Reading rosbag...")
        
        start_time_ns = None  # track timestamps for accurate sampling
        end_time_ns = None

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg = deserialize_message(data, self._tf_msg_type)

            # Feed transforms into the buffer
            for transform in msg.transforms:
                if topic == '/tf_static':
                    self._tf_buffer.set_transform_static(transform, 'path_extractor')
                else:
                    self._tf_buffer.set_transform(transform, 'path_extractor')
                    
                    # Track actual simulation timeline from frame headers
                    stamp = transform.header.stamp
                    t_ns = stamp.sec * 1000000000 + stamp.nanosec
                    if start_time_ns is None or t_ns < start_time_ns:
                        start_time_ns = t_ns
                    if end_time_ns is None or t_ns > end_time_ns:
                        end_time_ns = t_ns

        if start_time_ns is None or end_time_ns is None:
            self.get_logger().error("No dynamic transforms found in the bag file!")
            return

        self.get_logger().info(f"Extracting trajectory from sim time {start_time_ns*1e-9:.2f} to {end_time_ns*1e-9:.2f}...")

        # Accumulate and save path data
        current_time_ns = start_time_ns
        while current_time_ns <= end_time_ns:
            sec = current_time_ns // 1000000000
            nanosec = current_time_ns % 1000000000
            query_time = Time(seconds=sec, nanoseconds=nanosec)

            try:
                trans = self._tf_buffer.lookup_transform(
                    self._base_frame_id,
                    self._child_frame_id,
                    query_time
                )

                p = trans.transform.translation
                q = trans.transform.rotation

                self._path_data.append([
                    p.x, p.y, p.z,
                    q.x, q.y, q.z, q.w,
                    sec + nanosec * 1e-9
                ])

            except TransformException:
                # Dont fail if a transform is missing at a certain timestamp, just skip
                pass

            current_time_ns += self._sample_period_ns

        self._save_to_csv()


def main(args=None):
    # Define command line args
    parser = argparse.ArgumentParser()

    parser.add_argument(
        'bag_path', type=str,  # required
        help="Path to the rosbag directory"
    )

    parser.add_argument(
        '--base', type=str, default=BASE_FRAME_ID,
        help=f"Base frame ID (default: {BASE_FRAME_ID})"
    )
    parser.add_argument(
        '--child', type=str, default=CHILD_FRAME_ID,
        help=f"Child frame ID (default: {CHILD_FRAME_ID})"
    )
    parser.add_argument(
        '--out', type=str, default=OUT_FILENAME,
        help=f"Output CSV filename (default: {OUT_FILENAME})"
    )
    parser.add_argument(
        '--freq', type=float, default=PUB_FREQ,
        help=f"Sampling interval in seconds (default: {PUB_FREQ})"
    )
    
    # Parse user args and spin off ROS2 node
    parsed_args = parser.parse_args(args=args if args else sys.argv[1:])

    rclpy.init()
    node = PathExtractor(
        parsed_args.bag_path, 
        parsed_args.base, 
        parsed_args.child, 
        parsed_args.out, 
        parsed_args.freq
    )

    try:
        node.extract()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
