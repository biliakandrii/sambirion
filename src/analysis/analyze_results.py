#!/usr/bin/env python3
"""
Analyze Nav2 configuration comparison results.
Shows histograms and statistics for timing comparisons.
"""

import json
import pandas as pd
import numpy as np
from pathlib import Path
import argparse

# Set backend to non-interactive before importing pyplot
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def analyze_results(results_file, output_dir="plots"):
    # Load results
    with open(results_file, 'r') as f:
        data = json.load(f)
    
    df = pd.DataFrame(data)
    df = df[df['time'].notna()]  # Remove failed tests
    
    if len(df) == 0:
        print("No successful tests found!")
        return
    
    print("\n" + "="*70)
    print("NAV2 CONFIGURATION COMPARISON")
    print("="*70)
    
    # Statistics by config
    print("\nOverall Statistics:")
    print("-" * 70)
    for config in ['default', 'prediction']:
        config_data = df[df['nav2_config'] == config]['time']
        if len(config_data) > 0:
            print(f"\n{config.upper()}:")
            print(f"  Tests run:    {len(config_data)}")
            print(f"  Mean time:    {config_data.mean():.2f}s")
            print(f"  Std dev:      {config_data.std():.2f}s")
            print(f"  Min time:     {config_data.min():.2f}s")
            print(f"  Max time:     {config_data.max():.2f}s")
            print(f"  Median time:  {config_data.median():.2f}s")
    
    # Statistics by test
    print("\n" + "-" * 70)
    print("\nBy Test Scenario:")
    print("-" * 70)
    for test in df['test_name'].unique():
        print(f"\n{test}:")
        test_data = df[df['test_name'] == test]
        for config in ['default', 'prediction']:
            config_data = test_data[test_data['nav2_config'] == config]['time']
            if len(config_data) > 0:
                print(f"  {config:12s}: {config_data.mean():.2f}s ± {config_data.std():.2f}s  (n={len(config_data)})")
    
    print("\n" + "="*70 + "\n")
    
    # Create plots
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # 1. Combined histogram for both configs
    fig, ax = plt.subplots(figsize=(10, 6))
    
    default_data = df[df['nav2_config'] == 'default']['time']
    prediction_data = df[df['nav2_config'] == 'prediction']['time']
    
    # Determine common bins for both datasets
    all_data = pd.concat([default_data, prediction_data])
    bins = np.linspace(all_data.min(), all_data.max(), 20)
    
    if len(default_data) > 0:
        ax.hist(default_data, bins=bins, color='steelblue', edgecolor='black', 
               alpha=0.5, label=f'Default (mean: {default_data.mean():.2f}s, n={len(default_data)})')
    
    if len(prediction_data) > 0:
        ax.hist(prediction_data, bins=bins, color='coral', edgecolor='black', 
               alpha=0.5, label=f'Prediction (mean: {prediction_data.mean():.2f}s, n={len(prediction_data)})')
    
    # Add mean lines
    if len(default_data) > 0:
        ax.axvline(default_data.mean(), color='steelblue', linestyle='--', linewidth=2)
    if len(prediction_data) > 0:
        ax.axvline(prediction_data.mean(), color='coral', linestyle='--', linewidth=2)
    
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Frequency', fontsize=12)
    ax.set_title('Nav2 Configuration Time Distribution', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    hist_file = output_dir / 'histogram_comparison.png'
    plt.savefig(hist_file, dpi=300, bbox_inches='tight')
    print(f"Saved: {hist_file}")
    plt.close(fig)  # Close the figure explicitly
    
    # 2. Box plot comparison
    fig, ax = plt.subplots(figsize=(10, 6))
    
    default_data = df[df['nav2_config'] == 'default']['time']
    prediction_data = df[df['nav2_config'] == 'prediction']['time']
    
    bp = ax.boxplot([default_data, prediction_data], 
                     labels=['Default', 'Prediction'],
                     patch_artist=True,
                     widths=0.6)
    
    bp['boxes'][0].set_facecolor('steelblue')
    bp['boxes'][1].set_facecolor('coral')
    
    ax.set_ylabel('Time (seconds)', fontsize=12)
    ax.set_title('Nav2 Configuration Comparison', fontsize=14, fontweight='bold')
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    box_file = output_dir / 'boxplot_comparison.png'
    plt.savefig(box_file, dpi=300, bbox_inches='tight')
    print(f"Saved: {box_file}")
    plt.close(fig)  # Close the figure explicitly
    
    # 3. By test comparison
    tests = df['test_name'].unique()
    if len(tests) > 1:
        fig, ax = plt.subplots(figsize=(10, 6))
        
        x = np.arange(len(tests))
        width = 0.35
        
        default_means = [df[(df['test_name'] == test) & (df['nav2_config'] == 'default')]['time'].mean() 
                        for test in tests]
        prediction_means = [df[(df['test_name'] == test) & (df['nav2_config'] == 'prediction')]['time'].mean() 
                           for test in tests]
        
        default_std = [df[(df['test_name'] == test) & (df['nav2_config'] == 'default')]['time'].std() 
                      for test in tests]
        prediction_std = [df[(df['test_name'] == test) & (df['nav2_config'] == 'prediction')]['time'].std() 
                         for test in tests]
        
        ax.bar(x - width/2, default_means, width, label='Default', 
               color='steelblue', yerr=default_std, capsize=5)
        ax.bar(x + width/2, prediction_means, width, label='Prediction', 
               color='coral', yerr=prediction_std, capsize=5)
        
        ax.set_ylabel('Mean Time (seconds)', fontsize=12)
        ax.set_xlabel('Test Scenario', fontsize=12)
        ax.set_title('Mean Completion Time by Test', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(tests)
        ax.legend()
        ax.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        bar_file = output_dir / 'test_comparison.png'
        plt.savefig(bar_file, dpi=300, bbox_inches='tight')
        print(f"Saved: {bar_file}")
        plt.close(fig)  # Close the figure explicitly
    
    print(f"\nAll plots saved to: {output_dir}/")


def main():
    parser = argparse.ArgumentParser(description='Analyze Nav2 test results')
    parser.add_argument('results_file', help='Path to results JSON file')
    parser.add_argument('--output-dir', default='plots', help='Output directory for plots')
    args = parser.parse_args()
    
    analyze_results(args.results_file, args.output_dir)


if __name__ == '__main__':
    main()