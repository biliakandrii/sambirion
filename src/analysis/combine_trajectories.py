#!/usr/bin/env python3

"""
Script to combine multiple trajectory plots into one map showing all trajectories.
Extracts trajectory pixels from PNG images and overlays them on one map.
"""

import matplotlib.pyplot as plt
import numpy as np
import os
import glob
from PIL import Image
from scipy import ndimage


# ---------------------------------------------------------------------------
# Plot-area detection
# ---------------------------------------------------------------------------

def detect_plot_bounds(img):
    """
    Detect the pixel bounding box of the axes area by finding the five evenly-
    spaced tick marks on both axes (corresponding to world coords -4,-2,0,2,4).

    Returns (left, right, top, bottom) in pixel coordinates, or None on failure.
    """
    h, w = img.shape[:2]
    dark = img[:, :, 0] < 0.35  # works for near-black pixels on any channel

    def find_5_ticks_in_line(values):
        """Given a 1-D boolean array, find 5 evenly-spaced dark clusters."""
        dark_idx = np.where(values)[0]
        if len(dark_idx) < 5:
            return None
        # Group consecutive indices into clusters
        clusters = []
        g = [dark_idx[0]]
        for i in range(1, len(dark_idx)):
            if dark_idx[i] - dark_idx[i - 1] < 10:
                g.append(dark_idx[i])
            else:
                clusters.append(int(np.mean(g)))
                g = [dark_idx[i]]
        clusters.append(int(np.mean(g)))

        if len(clusters) != 5:
            return None
        diffs = np.diff(clusters)
        if np.std(diffs) / (np.mean(diffs) + 1e-9) < 0.05:
            return clusters
        return None

    # --- X-axis ticks: scan rows from bottom upward ---
    x_ticks = None
    for row in range(h - 1, h // 2, -1):
        result = find_5_ticks_in_line(dark[row, :])
        if result:
            x_ticks = result
            break

    # --- Y-axis ticks: scan columns from left ---
    y_ticks = None
    for col in range(0, w // 2):
        # Ignore title area (top 50 px)
        result = find_5_ticks_in_line(dark[50:, col])
        if result:
            y_ticks = [v + 50 for v in result]
            break

    if x_ticks is None or y_ticks is None:
        return None

    # Ticks are at world -4,-2,0,2,4 => 8 world-units span 4 tick-gaps
    # Extrapolate one more unit each side to reach +/-5 (the axis limit)
    x_scale = (x_ticks[-1] - x_ticks[0]) / 8.0   # px per world-unit
    y_scale = (y_ticks[-1] - y_ticks[0]) / 8.0

    left   = int(round(x_ticks[0]  - x_scale))
    right  = int(round(x_ticks[-1] + x_scale))
    top    = int(round(y_ticks[0]  - y_scale))
    bottom = int(round(y_ticks[-1] + y_scale))

    return left, right, top, bottom


# ---------------------------------------------------------------------------
# Blue trajectory extraction
# ---------------------------------------------------------------------------

def extract_trajectory_pixels(img, plot_bounds):
    """
    Extract blue trajectory pixels from within the plot area only.
    Small connected components (e.g. legend line sample) are discarded.

    Returns (x_px, y_px) arrays in full-image pixel coordinates.
    """
    left, right, top, bottom = plot_bounds

    # Crop to plot area, then mask out the top 10% where the legend lives
    plot_h = bottom - top
    legend_rows = int(plot_h * 0.10)   # legend is always in the top ~10%
    plot = img[top:bottom, left:right].copy()
    plot[:legend_rows, :] = 1.0        # blank out legend zone (set to white)

    # Blue detection: blue channel dominates and pixel isn't near-white
    blue_lead = (plot[:, :, 2].astype(float)
                 - np.maximum(plot[:, :, 0], plot[:, :, 1]).astype(float))
    brightness = (plot[:, :, 0].astype(float)
                  + plot[:, :, 1].astype(float)
                  + plot[:, :, 2].astype(float))

    blue_mask = (blue_lead > 0.25) & (brightness < 2.3)

    # Remove residual small components (JPEG noise blobs)
    labeled, n = ndimage.label(blue_mask)
    if n == 0:
        return np.array([]), np.array([])

    sizes = ndimage.sum(blue_mask, labeled, range(1, n + 1))
    keep = np.zeros(n + 1, dtype=bool)
    for i, s in enumerate(sizes):
        if s >= 50:
            keep[i + 1] = True

    filtered = keep[labeled]
    py_crop, px_crop = np.where(filtered)

    # Translate back to full-image coordinates
    return px_crop + left, py_crop + top


# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------

def pixels_to_world(px, py, plot_bounds, map_size=10.0):
    """Convert pixel coordinates to world (map) coordinates."""
    left, right, top, bottom = plot_bounds
    x_world = (px - left) / (right - left) * map_size - map_size / 2
    y_world = (1.0 - (py - top) / (bottom - top)) * map_size - map_size / 2
    return x_world, y_world


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def combine_trajectories(
    input_dir='./plots/trajectory/',
    output_file='combined_trajectories.png',
    map_size=10.0
):
    """
    Extract trajectories from PNG files and overlay them on one map.

    Args:
        input_dir:   Directory containing trajectory_*.png files.
        output_file: Output filename for combined plot.
        map_size:    Map size in metres (axes will span +/- map_size/2).
    """
    pattern   = os.path.join(input_dir, 'trajectory_*.png')
    png_files = sorted(glob.glob(pattern))

    if not png_files:
        print(f"No trajectory PNG files found in {input_dir}")
        return

    print(f"Found {len(png_files)} trajectory images")

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-map_size / 2,  map_size / 2)
    ax.set_ylim(-map_size / 2,  map_size / 2)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(
        f'Combined Robot Trajectories ({map_size:.0f}x{map_size:.0f} Map)\n'
        f'Total: {len(png_files)} trajectories',
        fontsize=14, fontweight='bold'
    )
    ax.set_aspect('equal')
    ax.plot(0, 0, 'go', markersize=10, label='Origin', zorder=5)

    for png_file in png_files:
        filename = os.path.basename(png_file)
        print(f"Processing: {filename}")

        # Load as float32 RGB in [0, 1]
        try:
            raw = Image.open(png_file).convert('RGB')
            img = np.array(raw).astype(np.float32) / 255.0
        except Exception as e:
            print(f"  Skipping {filename}: cannot open image ({e})")
            continue

        # Detect axes boundaries from tick marks
        bounds = detect_plot_bounds(img)
        if bounds is None:
            print(f"  WARNING: could not detect plot bounds, skipping {filename}")
            continue
        left, right, top, bottom = bounds
        print(f"  Plot bounds: left={left}, right={right}, top={top}, bottom={bottom}")

        # Extract blue trajectory pixels (inside plot area, small clusters removed)
        px, py = extract_trajectory_pixels(img, bounds)

        if len(px) == 0:
            print(f"  WARNING: no trajectory pixels found in {filename}")
            continue

        # Convert to world coordinates
        x_world, y_world = pixels_to_world(px, py, bounds, map_size)

        label = filename.replace('trajectory_', '').replace('.png', '')
        ax.scatter(x_world, y_world,
                   c='blue', s=2, alpha=0.6,
                   linewidths=0, label=label, rasterized=True)

        print(f"  Extracted {len(px)} trajectory pixels  "
              f"x=[{x_world.min():.2f}, {x_world.max():.2f}]  "
              f"y=[{y_world.min():.2f}, {y_world.max():.2f}]")

    ax.legend(loc='upper right', fontsize=8)
    plt.tight_layout()
    output_path = os.path.join(input_dir, output_file)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"\nCombined plot saved: {output_path}")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description='Combine trajectory plots into one map'
    )
    parser.add_argument('--input-dir',   default='./plots/trajectory/',
                        help='Directory containing trajectory_*.png files')
    parser.add_argument('--output-file', default='combined_trajectories.png',
                        help='Output filename')
    parser.add_argument('--map-size',    type=float, default=10.0,
                        help='Map size in metres')

    args = parser.parse_args()
    combine_trajectories(args.input_dir, args.output_file, args.map_size)