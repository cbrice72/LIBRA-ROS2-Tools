#!/usr/bin/env python3

# Brief:
#   A utility script that reads a CSV file containing 3D trajectory data for a
#   camera (with orientation) and calculates various metrics.
#
# Usage:
#   python3 camera_path_analyzer.py /path/to/csv_file

import argparse
import os
import sys
from time import sleep, perf_counter

from math import tan, radians
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from scipy.spatial import ConvexHull

from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# Realsense D456 intrinsics
CAMERA_MIN_OPERATING_DEPTH_M = 0.6
CAMERA_MAX_OPERATING_DEPTH_M = 6.0
CAMERA_FOV_H_DEG = 86.0  # horizontal
CAMERA_FOV_V_DEG = 57.0  # vertical


class PathAnalyzer:
    """
    End-to-end analysis of a moving camera trajectory.

    Calculates geometric, kinematic, and sensor-coverage metrics from a recorded
    camera path (nav_msgs/Path recorded into a CSV) with orientation awareness.
    """

    def __init__(self, csv_file, voxel_size=0.05, rays_per_pose=200):
        print(f"\n<< Running PathAnalyzer on {csv_file} >>\n")
        sleep(2)  # allow user to read printout

        # Known intrinsics
        self._depth_min = CAMERA_MIN_OPERATING_DEPTH_M
        self._depth_max = CAMERA_MAX_OPERATING_DEPTH_M
        self._fov_h = radians(CAMERA_FOV_H_DEG)
        self._fov_v = radians(CAMERA_FOV_V_DEG)

        # Params
        self._in_csv_filepath = csv_file
        self._voxel_size = voxel_size
        self._rays_per_pose = rays_per_pose

        # Internal properties
        self._positions = None
        self._rotations = None
        self._voxels = set()

        # Output metrics and filepath(s)
        self._total_path_length = 0.0
        self._workspace_bounds = None
        self._hull_volume = 0.0
        self._scanned_volume = 0.0
        self._scan_redundancy = 0.0

        csv_dir = os.path.dirname(os.path.abspath(self._in_csv_filepath))
        self._out_ply_filepath = os.path.join(csv_dir, "est_scan_volume.ply")

    def run(self):
        """
        Runs the full analysis pipeline.
        """
        t_start = perf_counter()

        self._load_csv()
        self._compute_path_metrics()
        self._compute_bounding_box()
        self._compute_convex_hull()
        self._estimate_scan_volume()

        """
        NOTE: The following points explain non-trivial metrics.

          - "Estimated scanned volume":
              This is an UPPER BOUND and ignores occlusion.

          - "Scan inefficiency (redundancy factor)":
              We get this from "R = total rays cast / unique voxels hit", where
              R>1 means overlapping views (higher = more revisits).
              Generally, 2-3 views per region is sufficient for quality coverage
              and anything beyond that adds little geometry (diminishing returns).
              This can be verified by rearranging the formula to get:
              "unique voxels hit = total rays cast / R" => "1/R", which gives
              the fraction of samples that hit new voxels; conversely, "1-(1/R)"
              gives the fraction of samples that don't provide new spatial info
              (note that this doesn't mean the redundant samples are useless).
              In a nutshell: it's a voxelized estimate of sensing redundancy
              based on Monte-Carlo frustrum sampling.
        """

        print("\n================================")
        print(f"Trajectory length: {self._total_path_length:.3f} m (w/ {len(self._positions)} samples)")
        print(f"Workspace envelope: X={self._workspace_bounds[0]:.3f} m, Y={self._workspace_bounds[1]:.3f} m, Z={self._workspace_bounds[2]:.3f} m")
        print(f"Volume traversed by camera: {self._hull_volume:.3f} m^3")
        print(f"Estimated scanned volume: {self._scanned_volume:.3f} m^3")
        print(f"  (mesh saved to: {self._out_ply_filepath})")
        print(f"Scan inefficiency (redundancy factor): {self._scan_redundancy:.2f}")
        print("================================\n")
        t_end = perf_counter()
        print(f"Script ran in {t_end - t_start:.3f} s\n")

        self._plot_scan_visualization()

    def _load_csv(self):
        """
        Loads nav_msgs/Path data from a CSV file.
        """
        print("[1/5] Loading nav_msgs/Path data from CSV... ", end="")

        try:
            # Extract positions and orientations
            df = pd.read_csv(self._in_csv_filepath)
            self._positions = df[['x', 'y', 'z']].to_numpy()
            quats = df[['qx', 'qy', 'qz', 'qw']].to_numpy()

            # Convert quaternions to scipy Rotations
            self._rotations = R.from_quat(quats)
        except FileNotFoundError:
            print("ERROR")
            print(f"    File not found: {self._in_csv_filepath}")
            exit(1)
        except Exception as e:
            print("ERROR")
            print(f"        Unexpected error: {e}")
            exit(1)

        print("SUCCESS")

    def _compute_path_metrics(self):
        """
        Computes total trajectory length based on segment distances.
        """
        print("[2/5] Computing traversed path length... ", end="")

        # Compute total path length by summing segment lengths
        diffs = np.diff(self._positions, axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)
        self._total_path_length = np.sum(segment_lengths)

        print("SUCCESS")

    def _compute_bounding_box(self):
        """
        Computes the axis-aligned bounding box of the Path.
        """
        print("[3/5] Computing axis-aligned workspace bounds... ", end="")

        # Batch-compute min and max along each axis
        min_xyz = self._positions.min(axis=0)
        max_xyz = self._positions.max(axis=0)
        self._workspace_bounds = max_xyz - min_xyz

        print("SUCCESS")

    def _compute_convex_hull(self):
        """
        Computes the convex hull created by the Path.
        """
        print("[4/5] Computing convex hull of trajectory points... ", end="")

        # Computing convex hull requires 4 or more non-coplanar points
        if len(self._positions) >= 4:
            hull = ConvexHull(self._positions)
            self._hull_volume = hull.volume
        else:
            print("ERROR")
            print(f"        Not enough points for convex hull: {len(self._positions)} (need >= 4)")

        print("SUCCESS")

    def _estimate_scan_volume(self):
        """
        Estimates the volume scanned by the camera using orientation-aware
        frustum ray sampling and voxelization.
        """
        print("[5/5] Estimating orientation-aware scanned volume... ", end="")

        # For each pose, sample rays within camera frustum
        for pos, rot in zip(self._positions, self._rotations):
            rays_cam = self._sample_frustum_rays(self._rays_per_pose)
            rays_world = rot.apply(rays_cam) + pos  # transform to world frame

            # Voxelize sampled points
            for p in rays_world:
                self._voxels.add(self._voxel_index(p))

        num_voxels = len(self._voxels)
        self._scanned_volume = num_voxels * (self._voxel_size ** 3)  # in m^3
        total_samples = len(self._positions) * self._rays_per_pose
        self._scan_redundancy = total_samples / max(num_voxels, 1)  # avoid dividing by zero

        print("SUCCESS")

    def _export_convex_hull_ply(self, points, faces):
        """
        Exports a triangular mesh to an ASCII PLY file.
        """
        print("  Saving convex hull mesh... ", end="")

        with open(self._out_ply_filepath, "w") as f:
            # Header
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write(f"element face {len(faces)}\n")
            f.write("property list uchar int vertex_indices\n")
            f.write("end_header\n")

            # Vertices
            for p in points:
                f.write(f"{p[0]} {p[1]} {p[2]}\n")

            # Faces (triangles)
            for tri in faces:
                f.write(f"3 {tri[0]} {tri[1]} {tri[2]}\n")

        print("SUCCESS")

    def _plot_scan_visualization(self):
        """
        Generates a visualization of the camera trajectory and its bounding box
        overlaid on the estimated scanned volume (from voxel hull).
        """

        print("Rendering 3D visualization... ", end="")

        if not self._voxels or self._positions is None:
            print("ERROR")
            print(f"    Missing voxel and/or position data")
            return
        
        print("")  # newline since there are multiple print statements in this function

        # Prep: Estimated scanned volume mesh
        voxels_np = np.array(list(self._voxels), dtype=float)
        voxel_points = (voxels_np + 0.5) * self._voxel_size
        hull = ConvexHull(voxel_points)
        hull_faces = voxel_points[hull.simplices]

        self._export_convex_hull_ply(voxel_points, hull.simplices)  # nice to have

        # Prep: Bounding box overlay
        min_xyz = self._positions.min(axis=0)
        max_xyz = self._positions.max(axis=0)

        bbox_edges = [
            # Bottom
            [min_xyz, [max_xyz[0], min_xyz[1], min_xyz[2]]],
            [min_xyz, [min_xyz[0], max_xyz[1], min_xyz[2]]],
            [[max_xyz[0], max_xyz[1], min_xyz[2]], [max_xyz[0], min_xyz[1], min_xyz[2]]],
            [[max_xyz[0], max_xyz[1], min_xyz[2]], [min_xyz[0], max_xyz[1], min_xyz[2]]],
            # Top
            [[min_xyz[0], min_xyz[1], max_xyz[2]], [max_xyz[0], min_xyz[1], max_xyz[2]]],
            [[min_xyz[0], min_xyz[1], max_xyz[2]], [min_xyz[0], max_xyz[1], max_xyz[2]]],
            [max_xyz, [max_xyz[0], min_xyz[1], max_xyz[2]]],
            [max_xyz, [min_xyz[0], max_xyz[1], max_xyz[2]]],
            # Walls
            [min_xyz, [min_xyz[0], min_xyz[1], max_xyz[2]]],
            [[max_xyz[0], min_xyz[1], min_xyz[2]], [max_xyz[0], min_xyz[1], max_xyz[2]]],
            [[min_xyz[0], max_xyz[1], min_xyz[2]], [min_xyz[0], max_xyz[1], max_xyz[2]]],
            [[max_xyz[0], max_xyz[1], min_xyz[2]], max_xyz],
        ]

        # Initialize 3D plot
        fig = plt.figure(figsize=(10, 9))
        ax = fig.add_subplot(111, projection='3d')

        # Plot: Scanned volume mesh
        mesh = Poly3DCollection(
            hull_faces,
            alpha=0.30,
            facecolor="orange",
            edgecolor="none"
        )
        ax.add_collection3d(mesh)

        # Plot: Bounding box
        for edge in bbox_edges:
            edge = np.array(edge)
            ax.plot(
                edge[:, 0],
                edge[:, 1],
                edge[:, 2],
                linestyle="--",
                color="blue",
                linewidth=3.0,
                alpha=0.9,
                zorder=5  # place above mesh
            )

        # Plot: Camera path
        ax.plot(
            self._positions[:, 0],
            self._positions[:, 1],
            self._positions[:, 2],
            color="cyan",
            linewidth=2.0,
            label="Camera Path",
            zorder=10  # place above all elements
        )

        # Plot: Start & End points
        ax.scatter(
            self._positions[0, 0], self._positions[0, 1], self._positions[0, 2],
            color="lime", s=100, marker="o", edgecolors="black", zorder=15, label="Start"
        )
        ax.scatter(
            self._positions[-1, 0], self._positions[-1, 1], self._positions[-1, -1],
            color="red", s=100, marker="o", edgecolors="black", zorder=15, label="End"
        )

        # Plot: Ensure everything is visible
        all_points = np.vstack([
            self._positions,
            voxel_points
        ])

        min_xyz = all_points.min(axis=0)
        max_xyz = all_points.max(axis=0)

        padding = 0.05 * (max_xyz - min_xyz)  # 5% padding

        ax.set_xlim(min_xyz[0] - padding[0], max_xyz[0] + padding[0])
        ax.set_ylim(min_xyz[1] - padding[1], max_xyz[1] + padding[1])
        ax.set_zlim(min_xyz[2] - padding[2], max_xyz[2] + padding[2])

        ax.set_box_aspect((1, 1, 1))

        # Plot: Formatting
        ax.set_title("Estimated Maximum Scanned Volume")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_box_aspect([1, 1, 1])

        legend_handles = [
            Line2D(
                [0], [0],
                color="cyan",
                linewidth=2.0,
                label="Camera trajectory"
            ),
            Line2D(
                [0], [0],
                marker="o", color="none", markerfacecolor="lime", markeredgecolor="black",
                markersize=10, label="Trajectory Start"
            ),
            Line2D(
                [0], [0],
                marker="o", color="none", markerfacecolor="red", markeredgecolor="black",
                markersize=10, label="Trajectory End"
            ),
            Line2D(
                [0], [0],
                color="blue",
                linestyle="--",
                linewidth=3.0,
                label="Camera trajectory bounding box"
            ),
            Patch(
                facecolor="orange",
                edgecolor="none",
                alpha=0.30,
                label="Estimated max. scanned volume"
            ),
        ]
        ax.legend(handles=legend_handles, loc="upper right")

        # Plot: In-graph labelling (est. scanned volume mesh)
        dx, dy, dz = max_xyz - min_xyz
        center = 0.5 * (min_xyz + max_xyz)

        ax.text(
            max_xyz[0], center[1], center[2],
            f"X = {dx:.2f} m",
            color="red",
            fontsize=11,
            ha="left",
            va="center"
        )
        ax.text(
            center[0], max_xyz[1], center[2],
            f"Y = {dy:.2f} m",
            color="red",
            fontsize=11,
            ha="center",
            va="bottom"
        )
        ax.text(
            center[0], center[1], max_xyz[2],
            f"Z = {dz:.2f} m",
            color="red",
            fontsize=11,
            ha="center",
            va="bottom"
        )

        # Generate plot
        plt.tight_layout()
        print("  Displaying plot (close window to continue)")
        plt.show()  # NOTE: blocking call

    def _sample_frustum_rays(self, n):
        """
        Utility function.
        Monte-Carlo samples points within the camera depth frustum
        (i.e., camera optical frame).
        """
        rays = []
        for _ in range(n):
            # Sample depth and normalized image plane coords
            d = np.random.uniform(self._depth_min, self._depth_max)
            u = np.random.uniform(-1.0, 1.0)
            v = np.random.uniform(-1.0, 1.0)

            # Get 3D point in camera frame
            x = d
            y = d * tan(self._fov_h / 2) * u
            z = d * tan(self._fov_v / 2) * v

            rays.append([x, y, z])

        return np.array(rays)

    def _voxel_index(self, point):
        """
        Utility function.
        Converts a 3D point to an integer voxel index.
        """
        return tuple((point / self._voxel_size).astype(int))


def main(args=None):
    # Define command line args
    parser = argparse.ArgumentParser()

    parser.add_argument(
        'csv_path', type=str,  # required
        help="Path to the CSV file containing trajectory data"
    )
    
    # Parse user args and run visualization pipeline
    parsed_args = parser.parse_args(args=args if args else sys.argv[1:])
    analyzer = PathAnalyzer(csv_file=parsed_args.csv_path)
    analyzer.run()

if __name__ == "__main__":
    main()
