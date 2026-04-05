#!/usr/bin/env python3
"""
Analyze Nav2 configuration comparison results.
Supports arbitrary configurations via `config_name`.
Generates statistics, histograms, boxplots, and per-test comparisons.
"""

import json
import pandas as pd
import numpy as np
from pathlib import Path
import argparse
import math

# Use non-interactive backend
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze_results(results_file, output_dir="plots"):
    # ------------------------------------------------------------------
    # Load results (keep ALL rows, including failures)
    # ------------------------------------------------------------------
    with open(results_file, "r") as f:
        data = json.load(f)

    df_all = pd.DataFrame(data)   # includes failed rows (time == null/NaN)
    df = df_all[df_all["time"].notna()].copy()  # successful only

    if "config_name" not in df_all.columns:
        raise ValueError("Input JSON must contain 'config_name' field")

    configs = sorted(df_all["config_name"].unique())
    tests = df_all["test_name"].unique()

    print("\n" + "=" * 70)
    print("NAV2 CONFIGURATION COMPARISON")
    print("=" * 70)

    # ------------------------------------------------------------------
    # Overall statistics (successful runs only)
    # ------------------------------------------------------------------
    print("\nOverall Statistics:")
    print("-" * 70)

    for config in configs:
        cfg_data = df[df["config_name"] == config]["time"]
        cfg_all = df_all[df_all["config_name"] == config]
        n_total = len(cfg_all)
        n_ok = len(cfg_data)
        print(f"\n{config}:")
        print(f"  Tests run:    {n_total}  (success={n_ok}, fail={n_total - n_ok})")
        if n_ok:
            print(f"  Mean time:    {cfg_data.mean():.2f}s")
            print(f"  Std dev:      {cfg_data.std():.2f}s")
            print(f"  Min time:     {cfg_data.min():.2f}s")
            print(f"  Max time:     {cfg_data.max():.2f}s")
            print(f"  Median time:  {cfg_data.median():.2f}s")

    # ------------------------------------------------------------------
    # Statistics by test scenario
    # ------------------------------------------------------------------
    print("\n" + "-" * 70)
    print("\nBy Test Scenario:")
    print("-" * 70)

    for test in tests:
        print(f"\n{test}:")
        test_df = df[df["test_name"] == test]

        for config in configs:
            cfg_data = test_df[test_df["config_name"] == config]["time"]
            if len(cfg_data) > 0:
                print(
                    f"  {config:20s}: "
                    f"{cfg_data.mean():.2f}s ± {cfg_data.std():.2f}s "
                    f"(n={len(cfg_data)})"
                )

    print("\n" + "=" * 70 + "\n")

    # ------------------------------------------------------------------
    # Plot setup
    # ------------------------------------------------------------------
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)

    colors = plt.cm.tab10.colors

    # ------------------------------------------------------------------
    # 1. Combined histogram comparison
    # ------------------------------------------------------------------
    if len(df) > 0:
        fig, ax = plt.subplots(figsize=(10, 6))

        all_times = df["time"]
        bins = np.linspace(all_times.min(), all_times.max(), 20)

        for i, config in enumerate(configs):
            cfg_data = df[df["config_name"] == config]["time"]
            if len(cfg_data) == 0:
                continue
            ax.hist(
                cfg_data,
                bins=bins,
                alpha=0.5,
                edgecolor="black",
                color=colors[i % len(colors)],
                label=f"{config} (mean={cfg_data.mean():.2f}s, n={len(cfg_data)})",
            )
            ax.axvline(
                cfg_data.mean(),
                color=colors[i % len(colors)],
                linestyle="--",
                linewidth=2,
            )

        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Frequency")
        ax.set_title("Nav2 Configuration Time Distribution", fontweight="bold")
        ax.legend()
        ax.grid(axis="y", alpha=0.3)

        plt.tight_layout()
        hist_file = output_dir / "histogram_comparison.png"
        plt.savefig(hist_file, dpi=300, bbox_inches="tight")
        print(f"Saved: {hist_file}")
        plt.close(fig)

    # ------------------------------------------------------------------
    # 2. Per-config histograms (same bins as combined)
    # ------------------------------------------------------------------
    if len(df) > 0:
        n_cols = min(2, len(configs))
        n_rows = math.ceil(len(configs) / n_cols)
        fig, axes = plt.subplots(
            n_rows, n_cols,
            figsize=(7 * n_cols, 5 * n_rows),
            squeeze=False,
        )

        all_times = df["time"]
        bins = np.linspace(all_times.min(), all_times.max(), 20)

        for idx, config in enumerate(configs):
            row, col = divmod(idx, n_cols)
            ax = axes[row][col]
            cfg_data = df[df["config_name"] == config]["time"]

            if len(cfg_data) == 0:
                ax.set_visible(False)
                continue

            ax.hist(
                cfg_data,
                bins=bins,
                alpha=0.75,
                edgecolor="black",
                color=colors[idx % len(colors)],
            )
            ax.axvline(
                cfg_data.mean(),
                color="black",
                linestyle="--",
                linewidth=1.8,
                label=f"mean={cfg_data.mean():.2f}s",
            )
            ax.axvline(
                cfg_data.median(),
                color="grey",
                linestyle=":",
                linewidth=1.8,
                label=f"median={cfg_data.median():.2f}s",
            )
            ax.set_title(config, fontweight="bold")
            ax.set_xlabel("Time (seconds)")
            ax.set_ylabel("Frequency")
            ax.legend(fontsize=8)
            ax.grid(axis="y", alpha=0.3)

        # Hide any unused subplot cells
        for idx in range(len(configs), n_rows * n_cols):
            row, col = divmod(idx, n_cols)
            axes[row][col].set_visible(False)

        fig.suptitle(
            "Per-Config Time Distribution (shared bins)",
            fontweight="bold",
            fontsize=13,
            y=1.01,
        )
        plt.tight_layout()
        per_cfg_hist_file = output_dir / "histogram_per_config.png"
        plt.savefig(per_cfg_hist_file, dpi=300, bbox_inches="tight")
        print(f"Saved: {per_cfg_hist_file}")
        plt.close(fig)

    # ------------------------------------------------------------------
    # 3. Boxplot comparison
    # ------------------------------------------------------------------
    if len(df) > 0:
        fig, ax = plt.subplots(figsize=(10, 6))

        box_data = [df[df["config_name"] == c]["time"] for c in configs]
        bp = ax.boxplot(
            box_data,
            labels=configs,
            patch_artist=True,
            widths=0.6,
        )

        for patch, color in zip(bp["boxes"], colors):
            patch.set_facecolor(color)

        ax.set_ylabel("Time (seconds)")
        ax.set_title("Nav2 Configuration Comparison", fontweight="bold")
        ax.grid(axis="y", alpha=0.3)

        plt.tight_layout()
        box_file = output_dir / "boxplot_comparison.png"
        plt.savefig(box_file, dpi=300, bbox_inches="tight")
        print(f"Saved: {box_file}")
        plt.close(fig)

    # ------------------------------------------------------------------
    # 4. Success rate — overall
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Left: stacked bar (count)
    ax = axes[0]
    success_counts = []
    fail_counts = []
    for config in configs:
        cfg_all = df_all[df_all["config_name"] == config]
        success_counts.append(cfg_all["time"].notna().sum())
        fail_counts.append(cfg_all["time"].isna().sum())

    x = np.arange(len(configs))
    bars_ok = ax.bar(x, success_counts, color=[colors[i % len(colors)] for i in range(len(configs))],
                     label="Success", edgecolor="black")
    bars_fail = ax.bar(x, fail_counts, bottom=success_counts,
                       color="lightcoral", label="Failure", edgecolor="black", hatch="//")

    ax.set_xticks(x)
    ax.set_xticklabels(configs, rotation=15, ha="right")
    ax.set_ylabel("Number of runs")
    ax.set_title("Success vs Failure Count\n(Overall)", fontweight="bold")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    # Annotate totals
    for i, (ok, fail) in enumerate(zip(success_counts, fail_counts)):
        total = ok + fail
        ax.text(i, total + 0.2, f"{ok}/{total}", ha="center", va="bottom", fontsize=9, fontweight="bold")

    # Right: success rate %
    ax = axes[1]
    rates = []
    for config in configs:
        cfg_all = df_all[df_all["config_name"] == config]
        n_total = len(cfg_all)
        n_ok = cfg_all["time"].notna().sum()
        rates.append(100.0 * n_ok / n_total if n_total else 0.0)

    bars = ax.bar(
        x, rates,
        color=[colors[i % len(colors)] for i in range(len(configs))],
        edgecolor="black",
    )
    ax.set_xticks(x)
    ax.set_xticklabels(configs, rotation=15, ha="right")
    ax.set_ylabel("Success rate (%)")
    ax.set_ylim(0, 110)
    ax.set_title("Success Rate (%)\n(Overall)", fontweight="bold")
    ax.axhline(100, color="green", linestyle="--", linewidth=1, alpha=0.5)
    ax.grid(axis="y", alpha=0.3)

    for bar, rate in zip(bars, rates):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 1.5,
            f"{rate:.1f}%",
            ha="center", va="bottom", fontsize=10, fontweight="bold",
        )

    plt.tight_layout()
    success_file = output_dir / "success_rate_overall.png"
    plt.savefig(success_file, dpi=300, bbox_inches="tight")
    print(f"Saved: {success_file}")
    plt.close(fig)

    # ------------------------------------------------------------------
    # 5. Success rate — per test scenario (if multiple tests)
    # ------------------------------------------------------------------
    if len(tests) > 1:
        fig, ax = plt.subplots(figsize=(max(10, len(tests) * 2), 6))

        x = np.arange(len(tests))
        width = 0.8 / len(configs)

        for i, config in enumerate(configs):
            rates = []
            for test in tests:
                subset = df_all[(df_all["test_name"] == test) & (df_all["config_name"] == config)]
                n_total = len(subset)
                n_ok = subset["time"].notna().sum()
                rates.append(100.0 * n_ok / n_total if n_total else 0.0)

            bars = ax.bar(
                x + i * width,
                rates,
                width,
                label=config,
                color=colors[i % len(colors)],
                edgecolor="black",
            )

            for bar, rate in zip(bars, rates):
                if rate < 100:  # only annotate non-perfect to reduce clutter
                    ax.text(
                        bar.get_x() + bar.get_width() / 2,
                        bar.get_height() + 1,
                        f"{rate:.0f}%",
                        ha="center", va="bottom", fontsize=7,
                    )

        ax.set_xticks(x + width * (len(configs) - 1) / 2)
        ax.set_xticklabels(tests, rotation=15, ha="right")
        ax.set_ylabel("Success rate (%)")
        ax.set_ylim(0, 115)
        ax.set_title("Success Rate (%) by Test Scenario", fontweight="bold")
        ax.legend()
        ax.axhline(100, color="green", linestyle="--", linewidth=1, alpha=0.5)
        ax.grid(axis="y", alpha=0.3)

        plt.tight_layout()
        success_test_file = output_dir / "success_rate_per_test.png"
        plt.savefig(success_test_file, dpi=300, bbox_inches="tight")
        print(f"Saved: {success_test_file}")
        plt.close(fig)

    # ------------------------------------------------------------------
    # 6. Mean time per test (bar chart)
    # ------------------------------------------------------------------
    if len(tests) > 1 and len(df) > 0:
        fig, ax = plt.subplots(figsize=(10, 6))

        x = np.arange(len(tests))
        width = 0.8 / len(configs)

        for i, config in enumerate(configs):
            means = [
                df[(df["test_name"] == test) & (df["config_name"] == config)]["time"].mean()
                for test in tests
            ]
            stds = [
                df[(df["test_name"] == test) & (df["config_name"] == config)]["time"].std()
                for test in tests
            ]

            ax.bar(
                x + i * width,
                means,
                width,
                yerr=stds,
                capsize=4,
                label=config,
                color=colors[i % len(colors)],
            )

        ax.set_xticks(x + width * (len(configs) - 1) / 2)
        ax.set_xticklabels(tests, rotation=15)
        ax.set_ylabel("Mean Time (seconds)")
        ax.set_xlabel("Test Scenario")
        ax.set_title("Mean Completion Time by Test", fontweight="bold")
        ax.legend()
        ax.grid(axis="y", alpha=0.3)

        plt.tight_layout()
        bar_file = output_dir / "test_comparison.png"
        plt.savefig(bar_file, dpi=300, bbox_inches="tight")
        print(f"Saved: {bar_file}")
        plt.close(fig)

    print(f"\nAll plots saved to: {output_dir}/")


def main():
    parser = argparse.ArgumentParser(description="Analyze Nav2 test results")
    parser.add_argument("results_file", help="Path to results JSON file")
    parser.add_argument(
        "--output-dir",
        default="plots",
        help="Output directory for plots",
    )
    args = parser.parse_args()

    analyze_results(args.results_file, args.output_dir)


if __name__ == "__main__":
    main()