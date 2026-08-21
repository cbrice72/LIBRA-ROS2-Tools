#!/usr/bin/env python3

# Brief:
#   A utility script that reads a CSV file containing 3D trajectory data and
#   generates 2D and 3D visualizations of the path.
#
# Usage:
#   python3 path_visualizer.py /path/to/csv_file
#
# Optional Args:
#   --gradient-2d GRADIENT_TYPE (one of: "time", "height")
#   --gradient-3d GRADIENT_TYPE
#   --cmap-2d CMAP_NAME (one of: "viridis", "plasma", "inferno", "magma", "cividis")
#   --cmap-3d CMAP_NAME

import argparse
import os
import sys

import numpy as np
import pandas as pd

from matplotlib.collections import LineCollection
import matplotlib.colors as mcolors
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from mpl_toolkits.mplot3d.art3d import Line3DCollection


CMAP_CHOICES = [  # NOTE: case sensitive!!
    "Blues_r", "Greens_r", "Greys_r", "Oranges_r", "Purples_r", "Reds_r",  # single hues
    "viridis", "plasma", "inferno", "magma", "cividis",  # "perceptually uniform sequential"
    "copper",  "cool",  "winter"  # others
]


class PathVisualizer:
    """
    Visualization utility for a recorded trajectory.

    Use `path_extractor.py` to generate the required CSV file from a rosbag's TF data.
    """

    def __init__(self, csv_file, gradient_type_2d=None, gradient_type_3d=None,
             cmap_2d="plasma", cmap_3d="plasma", solid_color="royalblue"):
        # Validate data
        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"CSV file not found: {csv_file}")
        
        self._df = pd.read_csv(csv_file)

        required_cols = {"x", "y", "z"}
        if not required_cols.issubset(self._df.columns):
            raise ValueError(f"CSV does not contain required columns: {required_cols}")
        
        # Store params
        self._csv_file = csv_file
        self._gradient_type_2d = gradient_type_2d
        self._gradient_type_3d = gradient_type_3d
        self._cmap_2d = cmap_2d  # Matplotlib colormap name for the 2D plot gradient
        self._cmap_3d = cmap_3d  # Matplotlib colormap name for the 3D plot gradient
        self._solid_color = solid_color  # Matplotlib color string for solid color rendering

        # Load trajectory data
        raw_x = self._df["x"].to_numpy()  
        raw_y = self._df["y"].to_numpy()
        self._x = -raw_y  # rotate axes to match...
        self._y = raw_x   # ... ROS coordinate frame
        self._z = self._df["z"].to_numpy()
        self._len = len(self._x)
        self._time = self._df["stamp_sec"] - self._df["stamp_sec"].min()  # elapsed time

        if self._len < 2:
            raise ValueError("Trajectory must contain two or more points!")

    def _initialize_plots(self) -> tuple[plt.Figure, plt.Axes, plt.Figure, plt.Axes]:
        """
        Initializes the 2D and 3D plots with appropriate titles, labels, and axis limits. Returns the figure and axes objects for both plots.
        """
        # Calculate axis bounds to minimize whitespace
        padding = 0.05

        xlim = (self._x.min(), self._x.max())
        ylim = (self._y.min(), self._y.max())
        zlim = (self._z.min(), self._z.max())
        
        dx = (xlim[1] - xlim[0]) * padding if xlim[1] != xlim[0] else 0.1
        dy = (ylim[1] - ylim[0]) * padding if ylim[1] != ylim[0] else 0.1
        dz = (zlim[1] - zlim[0]) * padding if zlim[1] != zlim[0] else 0.1

        # 2D plot (X-Y projection)
        fig2d = plt.figure("2D Trajectory Window", figsize=(8, 7))

        ax2d = fig2d.add_subplot(1, 1, 1)
        ax2d.set_title("2D Trajectory Projection (X-Y)", fontsize=12, fontweight="bold")
        ax2d.set_xlabel("X Position (m)")
        ax2d.set_ylabel("Y Position (m)")
        ax2d.grid(True, linestyle="--", alpha=0.6)
        
        ax2d.set_xlim(xlim[0] - dx, xlim[1] + dx)
        ax2d.set_ylim(ylim[0] - dy, ylim[1] + dy)

        ax2d.set_aspect('equal', adjustable='box')  # ensure accurate spatial representation

        # 3D plot
        fig3d = plt.figure("3D Trajectory Window", figsize=(8, 7))

        ax3d = fig3d.add_subplot(1, 1, 1, projection="3d")
        ax3d.set_title("3D Trajectory Path", fontsize=12, fontweight="bold")
        ax3d.set_xlabel("X (m)")
        ax3d.set_ylabel("Y (m)")
        ax3d.set_zlabel("Z (m)")
        
        ax3d.set_xlim(xlim[0] - dx, xlim[1] + dx)
        ax3d.set_ylim(ylim[0] - dy, ylim[1] + dy)
        ax3d.set_zlim(zlim[0] - dz, zlim[1] + dz)

        ax3d.set_box_aspect(  # ensure accurate spatial representation
            (xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0])
        )

        # Since we rotate the axes in __init__ to match the ROS coordinate frame,
        # we flip the sign on the X-axis for the graphs to be aligned with reality
        flip_sign = FuncFormatter(lambda val, pos: f"{-val:g}")
        ax2d.xaxis.set_major_formatter(flip_sign)
        ax3d.xaxis.set_major_formatter(flip_sign)

        return fig2d, ax2d, fig3d, ax3d

    def _compute_segment_colors(self, gradient_type: str) -> np.ndarray:
        """
        Computes per-segment color values for a gradient line plot based on the given gradient type.
        """
        if gradient_type == "time":
            # Map colors based on the average time of each line segment
            time_segments = (self._time.values[:-1] + self._time.values[1:]) / 2.0
            return (time_segments - self._time.min()) / (self._time.max() - self._time.min())
        elif gradient_type == "height":
            # Map colors based on the average height (Z) of each line segment
            z_segments = (self._z[:-1] + self._z[1:]) / 2.0
            return (z_segments - self._z.min()) / (self._z.max() - self._z.min())

    def _plot_trajectories(self, ax2d: plt.Axes, ax3d: plt.Axes, cmap2d: mcolors.Colormap, cmap3d: mcolors.Colormap) -> None:
        """
        Plots trajectory in 2D and 3D, using each plot's own gradient/cmap settings.
        """
        # 2D trajectory
        if self._gradient_type_2d:
            points_2d = np.vstack([self._x, self._y]).T.reshape(-1, 1, 2)
            segments_2d = np.concatenate([points_2d[:-1], points_2d[1:]], axis=1)
            colors_2d = self._compute_segment_colors(self._gradient_type_2d)

            lc2d = LineCollection(segments_2d, array=colors_2d, cmap=cmap2d, linewidths=2)
            ax2d.add_collection(lc2d)
            ax2d.autoscale_view()
        else:
            ax2d.plot(self._x, self._y, color=self._solid_color, linewidth=2)

        # 3D trajectory
        if self._gradient_type_3d:
            points_3d = np.vstack([self._x, self._y, self._z]).T.reshape(-1, 1, 3)
            segments_3d = np.concatenate([points_3d[:-1], points_3d[1:]], axis=1)
            colors_3d = self._compute_segment_colors(self._gradient_type_3d)

            lc3d = Line3DCollection(segments_3d, array=colors_3d, cmap=cmap3d, linewidths=2)
            ax3d.add_collection(lc3d)
            ax3d.autoscale_view()
        else:
            ax3d.plot(self._x, self._y, self._z, color=self._solid_color, linewidth=2)

    def _build_legend_elements(self, path_color: str | tuple) -> list[Line2D]:
        """
        Builds the legend handle list (path, start, end) for a given path color.
        """
        return [
            Line2D([0], [0], color=path_color, lw=2, label='Path'),
            Line2D([0], [0], marker='o', color='w', label='Start', markerfacecolor='green', markersize=9),
            Line2D([0], [0], marker='o', color='w', label='End', markerfacecolor='red', markersize=9),
        ]

    def _add_annotations(self, fig2d: plt.Figure, ax2d: plt.Axes, fig3d: plt.Figure, ax3d: plt.Axes,
                      cmap2d: mcolors.Colormap, cmap3d: mcolors.Colormap) -> None:
        """
        Adds plot annotations such as start/end points, color bars, and legends.
        """
        # Key points (start & end)
        ax2d.scatter(self._x[0], self._y[0], color="green", s=60, zorder=5)
        ax3d.scatter(self._x[0], self._y[0], self._z[0], color="green", s=60, zorder=5)

        ax2d.scatter(self._x[-1], self._y[-1], color="red", s=60, zorder=5)
        ax3d.scatter(self._x[-1], self._y[-1], self._z[-1], color="red", s=60, zorder=5)

        # Color bar (2D plot only)
        if self._gradient_type_2d:
            if self._gradient_type_2d == "time":
                norm = plt.Normalize(vmin=self._time.min(), vmax=self._time.max())
                label = "Elapsed Time (s)"
            elif self._gradient_type_2d == "height":
                norm = plt.Normalize(vmin=self._z.min(), vmax=self._z.max())
                label = "Height / Z Position (m)"

            sm = plt.cm.ScalarMappable(cmap=cmap2d, norm=norm)
            sm.set_array([])
            cbar = fig2d.colorbar(sm, ax=ax2d, pad=0.05, shrink=0.8)
            cbar.set_label(label, rotation=270, labelpad=15)

        # Legend
        path_color_2d = cmap2d(0.6) if self._gradient_type_2d else self._solid_color
        path_color_3d = cmap3d(0.6) if self._gradient_type_3d else self._solid_color

        ax2d.legend(handles=self._build_legend_elements(path_color_2d), loc="best")
        ax3d.legend(handles=self._build_legend_elements(path_color_3d), loc="best")

        # Adjust layout to prevent clipping
        fig2d.tight_layout()
        fig3d.tight_layout()

    def _truncate_colormap(self, cmap: mcolors.Colormap, min_val: float = 0.0, max_val: float = 1.0, n: int = 256) -> mcolors.Colormap:
        """
        Returns a new colormap that samples a sub-range of an existing colormap.
        """
        return mcolors.LinearSegmentedColormap.from_list(
            f"{cmap.name}_trunc", cmap(np.linspace(min_val, max_val, n))
        )

    def generate_plots(self):
        """
        Generates 2D and 3D trajectory profiles.
        """
        cmap2d = self._truncate_colormap(plt.get_cmap(self._cmap_2d), min_val=0.0, max_val=1.0)
        cmap3d = self._truncate_colormap(plt.get_cmap(self._cmap_3d), min_val=0.0, max_val=1.0)

        # Generate
        fig2d, ax2d, fig3d, ax3d = self._initialize_plots()
        self._plot_trajectories(ax2d, ax3d, cmap2d, cmap3d)
        self._add_annotations(fig2d, ax2d, fig3d, ax3d, cmap2d, cmap3d)

        # Display 
        print("Displaying figures (close window to finish)")
        plt.show()


def main(args=None):
    # Define command line args
    parser = argparse.ArgumentParser()

    parser.add_argument(
        'csv_path', type=str,  # required
        help="Path to the CSV file containing trajectory data"
    )

    parser.add_argument(
        '--gradient-2d', type=str, default=None,
        choices=["time", "height"],
        help="Gradient type for the 2D plot"
    )
    parser.add_argument(
        '--gradient-3d', type=str, default=None,
        choices=["time", "height"],
        help="Gradient type for the 3D plot"
    )
    parser.add_argument(
        '--cmap-2d', type=str, default="viridis",
        choices=CMAP_CHOICES,
        help="Matplotlib colormap name for the 2D plot (default: viridis)"
    )
    parser.add_argument(
        '--cmap-3d', type=str, default="viridis",
        choices=CMAP_CHOICES,
        help="Matplotlib colormap name for the 3D plot (default: viridis)"
    )
    
    # Parse user args and run visualization pipeline
    parsed_args = parser.parse_args(args=args if args else sys.argv[1:])
        
    try:
        visualizer = PathVisualizer(
            csv_file=parsed_args.csv_path,
            gradient_type_2d=parsed_args.gradient_2d,
            gradient_type_3d=parsed_args.gradient_3d,
            cmap_2d=parsed_args.cmap_2d,
            cmap_3d=parsed_args.cmap_3d,
            solid_color="royalblue"
        )
        visualizer.generate_plots()
        
    except Exception as e:
        print(f"[ERROR] {e}")


if __name__ == '__main__':
    main()