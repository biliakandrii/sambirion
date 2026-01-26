#!/usr/bin/env python3

"""
Script to combine multiple trajectory plots into one map showing all trajectories
Extracts trajectory pixels from PNG images and overlays them on one map
"""

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import glob


def extract_trajectory_from_image(img):
    """
    Extract trajectory line (blue pixels) from image
    Returns binary mask of trajectory pixels
    """
    # Extract blue pixels - be very lenient to capture antialiased edges
    # Blue channel should be significantly higher than red and green
    blue_mask = (img[:, :, 2] > 0.2) & (img[:, :, 0] < 0.5) & (img[:, :, 1] < 0.5) & ((img[:, :, 2] - img[:, :, 0]) > 0.1)
    return blue_mask


def combine_trajectories(input_dir='./plots/trajectory/', output_file='combined_trajectories.png', map_size=10.0):
    """
    Extract trajectories from PNG files and overlay them on one map
    
    Args:
        input_dir: Directory containing trajectory_*.png files
        output_file: Output filename for combined plot
        map_size: Map size in meters
    """
    
    # Find all trajectory PNG files
    pattern = os.path.join(input_dir, 'trajectory_*.png')
    png_files = sorted(glob.glob(pattern))
    
    if not png_files:
        print(f"No trajectory PNG files found in {input_dir}")
        return
    
    print(f"Found {len(png_files)} trajectory images")
    
    # Create figure
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Set up the plot limits
    ax.set_xlim(-map_size/2, map_size/2)
    ax.set_ylim(-map_size/2, map_size/2)
    
    # Grid and labels
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(f'Combined Robot Trajectories (10x10 Map)\nTotal: {len(png_files)} trajectories', 
                 fontsize=14, fontweight='bold')
    ax.set_aspect('equal')
    
    # Add origin marker
    ax.plot(0, 0, 'go', markersize=10, label='Origin', zorder=5)
    
    # Read and overlay each trajectory
    for idx, png_file in enumerate(png_files):
        filename = os.path.basename(png_file)
        print(f"Processing: {filename}")
        
        # Load image
        img = mpimg.imread(png_file)
        
        # Extract blue trajectory pixels
        blue_mask = extract_trajectory_from_image(img)
        
        # Get coordinates of trajectory pixels
        y_coords, x_coords = np.where(blue_mask)
        
        if len(x_coords) > 0:
            # Convert pixel coordinates to map coordinates
            height, width = img.shape[:2]
            
            # Map pixel coordinates to world coordinates
            x_map = (x_coords / width) * map_size - map_size/2
            y_map = (1 - y_coords / height) * map_size - map_size/2
            
            # Plot trajectory in blue
            label = filename.replace('trajectory_', '').replace('.png', '')
            ax.scatter(x_map, y_map, c='blue', s=1, alpha=0.7, label=label)
            
            print(f"  Extracted {len(x_coords)} trajectory points")
    
    # Add legend
    ax.legend(loc='upper right', fontsize=8)
    
    # Save combined plot
    plt.tight_layout()
    output_path = os.path.join(input_dir, output_file)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"\nCombined plot saved: {output_path}")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Combine trajectory plots into one map')
    parser.add_argument('--input-dir', default='./plots/trajectory/',
                        help='Input directory containing trajectory PNG files')
    parser.add_argument('--output-file', default='combined_trajectories.png',
                        help='Output filename')
    parser.add_argument('--map-size', type=float, default=10.0,
                        help='Map size in meters')
    
    args = parser.parse_args()
    
    combine_trajectories(args.input_dir, args.output_file, args.map_size)