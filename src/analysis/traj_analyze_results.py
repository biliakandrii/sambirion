#!/usr/bin/env python3

"""
Trajectory Analysis and Visualization Script
Analyzes trajectory_data.json and generates statistical plots and reports
"""

import json
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime
import argparse
from typing import List, Dict, Optional


class TrajectoryAnalyzer:
    def __init__(self, json_filepath='./plots/trajectory/trajectory_data.json'):
        self.json_filepath = json_filepath
        self.data = None
        self.runs = []
        
    def load_data(self) -> bool:
        """Load trajectory data from JSON file"""
        if not os.path.exists(self.json_filepath):
            print(f"❌ Error: JSON file not found: {self.json_filepath}")
            return False
        
        try:
            with open(self.json_filepath, 'r') as f:
                self.data = json.load(f)
            self.runs = self.data.get('runs', [])
            print(f"✅ Loaded {len(self.runs)} runs from {self.json_filepath}")
            return len(self.runs) > 0
        except Exception as e:
            print(f"❌ Error loading JSON: {e}")
            return False
    
    def extract_metrics(self) -> Dict:
        """Extract key metrics from all runs"""
        path_lengths = []
        min_distances = []
        straight_line_distances = []
        path_deviations = []  # Percentage deviation from straight line
        
        for run in self.runs:
            metrics = run.get('metrics', {})
            
            # Path length
            path_length = metrics.get('total_path_length_m')
            if path_length is not None:
                path_lengths.append(path_length)
            
            # Minimum obstacle distance
            min_dist = metrics.get('min_obstacle_distance_m')
            if min_dist is not None:
                min_distances.append(min_dist)
            
            # Straight line distance
            straight_line = metrics.get('straight_line_distance_m')
            if straight_line is not None:
                straight_line_distances.append(straight_line)
                
                # Calculate deviation percentage
                if path_length is not None and straight_line > 0:
                    deviation = ((path_length - straight_line) / straight_line) * 100
                    path_deviations.append(deviation)
        
        return {
            'path_lengths': path_lengths,
            'min_distances': min_distances,
            'straight_line_distances': straight_line_distances,
            'path_deviations': path_deviations
        }
    
    def calculate_statistics(self, values: List[float]) -> Dict:
        """Calculate basic statistics for a list of values"""
        if not values:
            return {
                'mean': None,
                'median': None,
                'std': None,
                'min': None,
                'max': None,
                'count': 0
            }
        
        return {
            'mean': np.mean(values),
            'median': np.median(values),
            'std': np.std(values),
            'min': np.min(values),
            'max': np.max(values),
            'count': len(values)
        }
    
    def plot_histograms(self, output_path='./plots/trajectory/analysis_histograms.png'):
        """Create histograms for min obstacle distance and path length"""
        metrics = self.extract_metrics()
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Trajectory Metrics Analysis', fontsize=16, fontweight='bold', y=0.995)
        
        # 1. Path Length Histogram
        ax1 = axes[0, 0]
        if metrics['path_lengths']:
            stats = self.calculate_statistics(metrics['path_lengths'])
            
            n, bins, patches = ax1.hist(metrics['path_lengths'], bins=20, 
                                       color='steelblue', alpha=0.7, edgecolor='black')
            
            # Add mean line
            mean_val = stats['mean']
            ax1.axvline(mean_val, color='red', linestyle='--', linewidth=2, 
                       label=f'Mean: {mean_val:.3f}m')
            
            # Add median line
            median_val = stats['median']
            ax1.axvline(median_val, color='orange', linestyle='--', linewidth=2,
                       label=f'Median: {median_val:.3f}m')
            
            ax1.set_xlabel('Path Length (meters)', fontsize=12, fontweight='bold')
            ax1.set_ylabel('Frequency', fontsize=12, fontweight='bold')
            ax1.set_title(f'Path Length Distribution (n={stats["count"]})', 
                         fontsize=13, fontweight='bold')
            ax1.legend(fontsize=10)
            ax1.grid(True, alpha=0.3)
            
            # Add statistics text box
            textstr = f'μ = {stats["mean"]:.3f}m\nσ = {stats["std"]:.3f}m\nMin = {stats["min"]:.3f}m\nMax = {stats["max"]:.3f}m'
            props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
            ax1.text(0.98, 0.97, textstr, transform=ax1.transAxes, fontsize=10,
                    verticalalignment='top', horizontalalignment='right', bbox=props)
        else:
            ax1.text(0.5, 0.5, 'No path length data', ha='center', va='center',
                    transform=ax1.transAxes, fontsize=14)
        
        # 2. Minimum Obstacle Distance Histogram
        ax2 = axes[0, 1]
        if metrics['min_distances']:
            stats = self.calculate_statistics(metrics['min_distances'])
            
            n, bins, patches = ax2.hist(metrics['min_distances'], bins=20,
                                       color='coral', alpha=0.7, edgecolor='black')
            
            # Color code bins by safety level
            for i, patch in enumerate(patches):
                if bins[i] < 0.5:
                    patch.set_facecolor('red')
                elif bins[i] < 1.0:
                    patch.set_facecolor('orange')
                else:
                    patch.set_facecolor('green')
            
            # Add mean line
            mean_val = stats['mean']
            ax2.axvline(mean_val, color='darkred', linestyle='--', linewidth=2,
                       label=f'Mean: {mean_val:.3f}m')
            
            # Add median line
            median_val = stats['median']
            ax2.axvline(median_val, color='darkblue', linestyle='--', linewidth=2,
                       label=f'Median: {median_val:.3f}m')
            
            # Add safety zones
            ax2.axvline(0.5, color='red', linestyle=':', alpha=0.5, linewidth=1.5,
                       label='Danger zone (<0.5m)')
            ax2.axvline(1.0, color='orange', linestyle=':', alpha=0.5, linewidth=1.5,
                       label='Warning zone (<1.0m)')
            
            ax2.set_xlabel('Minimum Obstacle Distance (meters)', fontsize=12, fontweight='bold')
            ax2.set_ylabel('Frequency', fontsize=12, fontweight='bold')
            ax2.set_title(f'Min Obstacle Distance Distribution (n={stats["count"]})', 
                         fontsize=13, fontweight='bold')
            ax2.legend(fontsize=9)
            ax2.grid(True, alpha=0.3)
            
            # Add statistics text box
            textstr = f'μ = {stats["mean"]:.3f}m\nσ = {stats["std"]:.3f}m\nMin = {stats["min"]:.3f}m\nMax = {stats["max"]:.3f}m'
            props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
            ax2.text(0.98, 0.97, textstr, transform=ax2.transAxes, fontsize=10,
                    verticalalignment='top', horizontalalignment='right', bbox=props)
        else:
            ax2.text(0.5, 0.5, 'No obstacle distance data', ha='center', va='center',
                    transform=ax2.transAxes, fontsize=14)
        
        # 3. Path Deviation Histogram
        ax3 = axes[1, 0]
        if metrics['path_deviations']:
            stats = self.calculate_statistics(metrics['path_deviations'])
            
            n, bins, patches = ax3.hist(metrics['path_deviations'], bins=20,
                                       color='mediumpurple', alpha=0.7, edgecolor='black')
            
            # Add mean line
            mean_val = stats['mean']
            ax3.axvline(mean_val, color='red', linestyle='--', linewidth=2,
                       label=f'Mean: {mean_val:.1f}%')
            
            # Add median line
            median_val = stats['median']
            ax3.axvline(median_val, color='orange', linestyle='--', linewidth=2,
                       label=f'Median: {median_val:.1f}%')
            
            # Add ideal (0% deviation) line
            ax3.axvline(0, color='green', linestyle=':', linewidth=2, alpha=0.5,
                       label='Ideal (0% deviation)')
            
            ax3.set_xlabel('Path Deviation from Straight Line (%)', fontsize=12, fontweight='bold')
            ax3.set_ylabel('Frequency', fontsize=12, fontweight='bold')
            ax3.set_title(f'Path Efficiency Distribution (n={stats["count"]})', 
                         fontsize=13, fontweight='bold')
            ax3.legend(fontsize=10)
            ax3.grid(True, alpha=0.3)
            
            # Add statistics text box
            textstr = f'μ = {stats["mean"]:.1f}%\nσ = {stats["std"]:.1f}%\nMin = {stats["min"]:.1f}%\nMax = {stats["max"]:.1f}%'
            props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
            ax3.text(0.98, 0.97, textstr, transform=ax3.transAxes, fontsize=10,
                    verticalalignment='top', horizontalalignment='right', bbox=props)
        else:
            ax3.text(0.5, 0.5, 'No deviation data', ha='center', va='center',
                    transform=ax3.transAxes, fontsize=14)
        
        # 4. Summary Statistics Table
        ax4 = axes[1, 1]
        ax4.axis('off')
        
        # Prepare summary data
        summary_data = []
        
        if metrics['path_lengths']:
            stats_pl = self.calculate_statistics(metrics['path_lengths'])
            summary_data.append(['Path Length (m)', 
                               f"{stats_pl['mean']:.3f}", 
                               f"{stats_pl['std']:.3f}",
                               f"{stats_pl['min']:.3f}",
                               f"{stats_pl['max']:.3f}",
                               f"{stats_pl['count']}"])
        
        if metrics['min_distances']:
            stats_md = self.calculate_statistics(metrics['min_distances'])
            summary_data.append(['Min Obstacle Dist (m)', 
                               f"{stats_md['mean']:.3f}", 
                               f"{stats_md['std']:.3f}",
                               f"{stats_md['min']:.3f}",
                               f"{stats_md['max']:.3f}",
                               f"{stats_md['count']}"])
        
        if metrics['path_deviations']:
            stats_pd = self.calculate_statistics(metrics['path_deviations'])
            summary_data.append(['Path Deviation (%)', 
                               f"{stats_pd['mean']:.1f}", 
                               f"{stats_pd['std']:.1f}",
                               f"{stats_pd['min']:.1f}",
                               f"{stats_pd['max']:.1f}",
                               f"{stats_pd['count']}"])
        
        if metrics['straight_line_distances']:
            stats_sl = self.calculate_statistics(metrics['straight_line_distances'])
            summary_data.append(['Straight Line (m)', 
                               f"{stats_sl['mean']:.3f}", 
                               f"{stats_sl['std']:.3f}",
                               f"{stats_sl['min']:.3f}",
                               f"{stats_sl['max']:.3f}",
                               f"{stats_sl['count']}"])
        
        if summary_data:
            # Create table
            table = ax4.table(cellText=summary_data,
                            colLabels=['Metric', 'Mean', 'Std Dev', 'Min', 'Max', 'Count'],
                            cellLoc='center',
                            loc='center',
                            bbox=[0.0, 0.3, 1.0, 0.6])
            
            table.auto_set_font_size(False)
            table.set_fontsize(10)
            table.scale(1, 2)
            
            # Style header
            for i in range(6):
                table[(0, i)].set_facecolor('#4CAF50')
                table[(0, i)].set_text_props(weight='bold', color='white')
            
            # Color code rows
            colors = ['#E3F2FD', '#FFE0B2', '#F3E5F5', '#E8F5E9']
            for i in range(len(summary_data)):
                for j in range(6):
                    table[(i+1, j)].set_facecolor(colors[i % len(colors)])
            
            ax4.set_title('Summary Statistics', fontsize=13, fontweight='bold', pad=20)
        else:
            ax4.text(0.5, 0.5, 'No data available', ha='center', va='center',
                    transform=ax4.transAxes, fontsize=14)
        
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"✅ Saved: {output_path}")
    
    def plot_scatter_analysis(self, output_path='./plots/trajectory/analysis_scatter.png'):
        """Create scatter plots showing relationships between metrics"""
        metrics = self.extract_metrics()
        
        fig, axes = plt.subplots(1, 2, figsize=(16, 6))
        fig.suptitle('Metric Relationships Analysis', fontsize=16, fontweight='bold')
        
        # 1. Path Length vs Deviation
        ax1 = axes[0]
        if metrics['path_lengths'] and metrics['path_deviations']:
            # Align data (both should have same length if calculated correctly)
            valid_indices = []
            pl_values = []
            dev_values = []
            
            for i, run in enumerate(self.runs):
                m = run.get('metrics', {})
                pl = m.get('total_path_length_m')
                sl = m.get('straight_line_distance_m')
                
                if pl is not None and sl is not None and sl > 0:
                    dev = ((pl - sl) / sl) * 100
                    pl_values.append(pl)
                    dev_values.append(dev)
            
            if pl_values and dev_values:
                scatter = ax1.scatter(pl_values, dev_values, 
                                    c=dev_values, cmap='RdYlGn_r',
                                    s=100, alpha=0.6, edgecolors='black')
                
                # Add colorbar
                cbar = plt.colorbar(scatter, ax=ax1)
                cbar.set_label('Deviation (%)', fontsize=10)
                
                # Add trend line
                z = np.polyfit(pl_values, dev_values, 1)
                p = np.poly1d(z)
                x_trend = np.linspace(min(pl_values), max(pl_values), 100)
                ax1.plot(x_trend, p(x_trend), "r--", alpha=0.8, linewidth=2,
                        label=f'Trend: y={z[0]:.2f}x+{z[1]:.2f}')
                
                ax1.set_xlabel('Path Length (m)', fontsize=12, fontweight='bold')
                ax1.set_ylabel('Deviation from Straight Line (%)', fontsize=12, fontweight='bold')
                ax1.set_title('Path Length vs Path Deviation', fontsize=13, fontweight='bold')
                ax1.legend(fontsize=10)
                ax1.grid(True, alpha=0.3)
        else:
            ax1.text(0.5, 0.5, 'Insufficient data', ha='center', va='center',
                    transform=ax1.transAxes, fontsize=14)
        
        # 2. Min Distance vs Path Length
        ax2 = axes[1]
        if metrics['min_distances'] and metrics['path_lengths']:
            # Align data
            min_dist_values = []
            pl_values = []
            
            for run in self.runs:
                m = run.get('metrics', {})
                md = m.get('min_obstacle_distance_m')
                pl = m.get('total_path_length_m')
                
                if md is not None and pl is not None:
                    min_dist_values.append(md)
                    pl_values.append(pl)
            
            if min_dist_values and pl_values:
                # Color code by safety
                colors = ['red' if d < 0.5 else 'orange' if d < 1.0 else 'green' 
                         for d in min_dist_values]
                
                ax2.scatter(min_dist_values, pl_values,
                          c=colors, s=100, alpha=0.6, edgecolors='black')
                
                # Add safety zone backgrounds
                ax2.axvspan(0, 0.5, alpha=0.1, color='red', label='Danger zone')
                ax2.axvspan(0.5, 1.0, alpha=0.1, color='orange', label='Warning zone')
                ax2.axvspan(1.0, max(min_dist_values) if min_dist_values else 5, 
                           alpha=0.1, color='green', label='Safe zone')
                
                ax2.set_xlabel('Minimum Obstacle Distance (m)', fontsize=12, fontweight='bold')
                ax2.set_ylabel('Path Length (m)', fontsize=12, fontweight='bold')
                ax2.set_title('Min Obstacle Distance vs Path Length', fontsize=13, fontweight='bold')
                ax2.legend(fontsize=10)
                ax2.grid(True, alpha=0.3)
        else:
            ax2.text(0.5, 0.5, 'Insufficient data', ha='center', va='center',
                    transform=ax2.transAxes, fontsize=14)
        
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"✅ Saved: {output_path}")
    
    def generate_text_report(self, output_path='./plots/trajectory/analysis_report.txt'):
        """Generate detailed text report with statistics"""
        metrics = self.extract_metrics()
        
        with open(output_path, 'w') as f:
            f.write("=" * 80 + "\n")
            f.write("TRAJECTORY ANALYSIS REPORT\n")
            f.write("=" * 80 + "\n\n")
            
            f.write(f"Report Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Data Source: {self.json_filepath}\n")
            f.write(f"Total Runs Analyzed: {len(self.runs)}\n\n")
            
            # Path Length Statistics
            f.write("-" * 80 + "\n")
            f.write("PATH LENGTH STATISTICS\n")
            f.write("-" * 80 + "\n")
            if metrics['path_lengths']:
                stats = self.calculate_statistics(metrics['path_lengths'])
                f.write(f"Number of runs: {stats['count']}\n")
                f.write(f"Mean: {stats['mean']:.3f} meters\n")
                f.write(f"Median: {stats['median']:.3f} meters\n")
                f.write(f"Std Dev: {stats['std']:.3f} meters\n")
                f.write(f"Min: {stats['min']:.3f} meters\n")
                f.write(f"Max: {stats['max']:.3f} meters\n")
                f.write(f"Range: {stats['max'] - stats['min']:.3f} meters\n")
            else:
                f.write("No path length data available\n")
            f.write("\n")
            
            # Minimum Obstacle Distance Statistics
            f.write("-" * 80 + "\n")
            f.write("MINIMUM OBSTACLE DISTANCE STATISTICS\n")
            f.write("-" * 80 + "\n")
            if metrics['min_distances']:
                stats = self.calculate_statistics(metrics['min_distances'])
                f.write(f"Number of runs: {stats['count']}\n")
                f.write(f"Mean: {stats['mean']:.3f} meters\n")
                f.write(f"Median: {stats['median']:.3f} meters\n")
                f.write(f"Std Dev: {stats['std']:.3f} meters\n")
                f.write(f"Min: {stats['min']:.3f} meters\n")
                f.write(f"Max: {stats['max']:.3f} meters\n")
                f.write(f"Range: {stats['max'] - stats['min']:.3f} meters\n\n")
                
                # Safety analysis
                danger_count = sum(1 for d in metrics['min_distances'] if d < 0.5)
                warning_count = sum(1 for d in metrics['min_distances'] if 0.5 <= d < 1.0)
                safe_count = sum(1 for d in metrics['min_distances'] if d >= 1.0)
                
                f.write("Safety Analysis:\n")
                f.write(f"  Danger zone (<0.5m): {danger_count} runs ({danger_count/stats['count']*100:.1f}%)\n")
                f.write(f"  Warning zone (0.5-1.0m): {warning_count} runs ({warning_count/stats['count']*100:.1f}%)\n")
                f.write(f"  Safe zone (>1.0m): {safe_count} runs ({safe_count/stats['count']*100:.1f}%)\n")
            else:
                f.write("No obstacle distance data available\n")
            f.write("\n")
            
            # Path Deviation Statistics
            f.write("-" * 80 + "\n")
            f.write("PATH DEVIATION FROM STRAIGHT LINE STATISTICS\n")
            f.write("-" * 80 + "\n")
            if metrics['path_deviations']:
                stats = self.calculate_statistics(metrics['path_deviations'])
                f.write(f"Number of runs: {stats['count']}\n")
                f.write(f"Mean deviation: {stats['mean']:.1f}%\n")
                f.write(f"Median deviation: {stats['median']:.1f}%\n")
                f.write(f"Std Dev: {stats['std']:.1f}%\n")
                f.write(f"Min deviation: {stats['min']:.1f}%\n")
                f.write(f"Max deviation: {stats['max']:.1f}%\n")
                f.write(f"Range: {stats['max'] - stats['min']:.1f}%\n\n")
                
                # Efficiency analysis
                efficient_count = sum(1 for d in metrics['path_deviations'] if d < 10)
                moderate_count = sum(1 for d in metrics['path_deviations'] if 10 <= d < 30)
                inefficient_count = sum(1 for d in metrics['path_deviations'] if d >= 30)
                
                f.write("Efficiency Analysis:\n")
                f.write(f"  Efficient (<10% deviation): {efficient_count} runs ({efficient_count/stats['count']*100:.1f}%)\n")
                f.write(f"  Moderate (10-30% deviation): {moderate_count} runs ({moderate_count/stats['count']*100:.1f}%)\n")
                f.write(f"  Inefficient (>30% deviation): {inefficient_count} runs ({inefficient_count/stats['count']*100:.1f}%)\n")
            else:
                f.write("No path deviation data available\n")
            f.write("\n")
            
            # Individual Run Details
            f.write("-" * 80 + "\n")
            f.write("INDIVIDUAL RUN DETAILS\n")
            f.write("-" * 80 + "\n")
            for i, run in enumerate(self.runs, 1):
                m = run.get('metrics', {})
                f.write(f"\nRun {i}: {run.get('run_id', 'Unknown')}\n")
                f.write(f"  Path Length: {m.get('total_path_length_m', 'N/A')} m\n")
                f.write(f"  Straight Line: {m.get('straight_line_distance_m', 'N/A')} m\n")
                f.write(f"  Min Obstacle Distance: {m.get('min_obstacle_distance_m', 'N/A')} m\n")
                
                pl = m.get('total_path_length_m')
                sl = m.get('straight_line_distance_m')
                if pl is not None and sl is not None and sl > 0:
                    deviation = ((pl - sl) / sl) * 100
                    f.write(f"  Deviation: {deviation:.1f}%\n")
            
            f.write("\n" + "=" * 80 + "\n")
        
        print(f"✅ Saved: {output_path}")
    
    def generate_all_analysis(self, output_dir=None):
        """Generate all analysis outputs"""
        if not self.load_data():
            return
        
        if output_dir is None:
            output_dir = os.path.dirname(self.json_filepath)
        
        os.makedirs(output_dir, exist_ok=True)
        
        print(f"\n📊 Generating analysis for {len(self.runs)} runs...")
        print(f"Output directory: {os.path.abspath(output_dir)}\n")
        
        # Generate plots and reports
        self.plot_histograms(os.path.join(output_dir, 'analysis_histograms.png'))
        self.plot_scatter_analysis(os.path.join(output_dir, 'analysis_scatter.png'))
        self.generate_text_report(os.path.join(output_dir, 'analysis_report.txt'))
        
        # Print summary to console
        metrics = self.extract_metrics()
        print("\n" + "=" * 80)
        print("SUMMARY STATISTICS")
        print("=" * 80)
        
        if metrics['path_lengths']:
            stats = self.calculate_statistics(metrics['path_lengths'])
            print(f"\n📏 Path Length:")
            print(f"   Mean: {stats['mean']:.3f}m ± {stats['std']:.3f}m")
            print(f"   Range: {stats['min']:.3f}m - {stats['max']:.3f}m")
        
        if metrics['min_distances']:
            stats = self.calculate_statistics(metrics['min_distances'])
            print(f"\n🎯 Minimum Obstacle Distance:")
            print(f"   Mean: {stats['mean']:.3f}m ± {stats['std']:.3f}m")
            print(f"   Range: {stats['min']:.3f}m - {stats['max']:.3f}m")
        
        if metrics['path_deviations']:
            stats = self.calculate_statistics(metrics['path_deviations'])
            print(f"\n📈 Path Deviation from Straight Line:")
            print(f"   Mean: {stats['mean']:.1f}% ± {stats['std']:.1f}%")
            print(f"   Range: {stats['min']:.1f}% - {stats['max']:.1f}%")
        
        print("\n" + "=" * 80)
        print("✅ Analysis complete! Check the output directory for detailed results.\n")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze trajectory data and generate statistical plots'
    )
    parser.add_argument(
        '--json', 
        type=str, 
        default='./plots/trajectory/trajectory_data.json',
        help='Path to trajectory_data.json file'
    )
    parser.add_argument(
        '--output-dir', 
        type=str, 
        default=None,
        help='Output directory for plots and reports (default: same as JSON location)'
    )
    
    args = parser.parse_args()
    
    # Create analyzer and run analysis
    analyzer = TrajectoryAnalyzer(args.json)
    analyzer.generate_all_analysis(args.output_dir)


if __name__ == '__main__':
    main()