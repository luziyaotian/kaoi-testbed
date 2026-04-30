#!/usr/bin/env python3
"""
plot_experiment.py — quick-look plots for an AoI logger CSV.

Reads a CSV produced by aoi_logger and generates four PNG figures:

    1. aoi_timeline.png     — AoI(t) with profile changes annotated
    2. err_timeline.png     — per-joint sync error timeline
    3. err_by_profile.png   — boxplot of err_L2 per network profile
    4. aoi_vs_err.png       — scatter of AoI vs err_L2 coloured by profile
                              (this is the key figure for the paper)

Run after an experiment sweep:

    python3 plot_experiment.py ~/testbed_logs/aoi_20260420_123456.csv

By default outputs next to the CSV with suffix-based filenames. Override with
--out-dir if you want a different location.

Dependencies: pandas, matplotlib, numpy (install with pip if missing).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


N_JOINTS = 6


def load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Trim the warm-up transient: drop anything before the first non-nan aoi
    first_valid = df["aoi_s"].first_valid_index()
    if first_valid is not None:
        df = df.loc[first_valid:].reset_index(drop=True)
    # Time relative to experiment start
    df["t_rel"] = df["wall_time_s"] - df["wall_time_s"].iloc[0]
    # Convert to ms for more readable plots
    df["aoi_ms"] = df["aoi_s"] * 1000
    df["err_l2_mrad"] = df["err_l2_rad"] * 1000
    df["err_max_mrad"] = df["err_max_rad"] * 1000
    return df


def profile_spans(df: pd.DataFrame):
    """Compute (profile, t_start, t_end) tuples marking when each profile
    was active. Used for shaded regions in timeline plots."""
    spans = []
    if df.empty:
        return spans
    current = df["network_profile"].iloc[0]
    start = df["t_rel"].iloc[0]
    for i in range(1, len(df)):
        if df["network_profile"].iloc[i] != current:
            spans.append((current, start, df["t_rel"].iloc[i]))
            current = df["network_profile"].iloc[i]
            start = df["t_rel"].iloc[i]
    spans.append((current, start, df["t_rel"].iloc[-1]))
    return spans


def profile_colours(profiles):
    # Use a qualitative palette so profiles are distinguishable
    palette = plt.cm.tab10.colors
    return {name: palette[i % len(palette)]
            for i, name in enumerate(sorted(set(profiles)))}


def plot_aoi_timeline(df: pd.DataFrame, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 4))
    spans = profile_spans(df)
    colours = profile_colours([s[0] for s in spans])
    for name, t0, t1 in spans:
        ax.axvspan(t0, t1, color=colours[name], alpha=0.15)
        ax.text(0.5 * (t0 + t1), ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 1,
                name, rotation=90, ha="center", va="top", fontsize=7,
                alpha=0.7)
    ax.plot(df["t_rel"], df["aoi_ms"], linewidth=0.6, color="black")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("AoI (ms)")
    ax.set_title("Age of Information over time (shaded = network profile)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


def plot_err_timeline(df: pd.DataFrame, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 4))
    spans = profile_spans(df)
    colours = profile_colours([s[0] for s in spans])
    for name, t0, t1 in spans:
        ax.axvspan(t0, t1, color=colours[name], alpha=0.15)
    ax.plot(df["t_rel"], df["err_l2_mrad"], linewidth=0.6,
            color="tab:red", label="L2 norm")
    ax.plot(df["t_rel"], df["err_max_mrad"], linewidth=0.4,
            color="tab:blue", alpha=0.6, label="max(|err_i|)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Sync error (mrad)")
    ax.set_title("Twin ↔ robot joint-space sync error over time")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


def plot_err_by_profile(df: pd.DataFrame, out: Path) -> None:
    # Drop profile-transition transients (first 2 s of each profile)
    df2 = df.copy()
    trans_window_s = 2.0
    keep_mask = np.ones(len(df2), dtype=bool)
    spans = profile_spans(df2)
    for _name, t0, _ in spans:
        keep_mask &= ~((df2["t_rel"] >= t0) & (df2["t_rel"] < t0 + trans_window_s))
    df2 = df2[keep_mask]

    # Drop 'unknown' (before any profile message arrived)
    df2 = df2[df2["network_profile"] != "unknown"]

    order = (df2.groupby("network_profile")["err_l2_mrad"]
                 .median().sort_values().index.tolist())
    data = [df2.loc[df2["network_profile"] == p, "err_l2_mrad"].dropna().values
            for p in order]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.boxplot(data, tick_labels=order, showfliers=False)
    ax.set_xlabel("Network profile")
    ax.set_ylabel("Sync error L2 (mrad)")
    ax.set_title("Sync error by network profile (transients excluded)")
    ax.grid(True, axis="y", alpha=0.3)
    plt.xticks(rotation=20, ha="right")
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


def plot_aoi_vs_err(df: pd.DataFrame, out: Path) -> None:
    df2 = df[df["network_profile"] != "unknown"].copy()
    colours = profile_colours(df2["network_profile"].unique())

    fig, ax = plt.subplots(figsize=(8, 6))
    for profile, sub in df2.groupby("network_profile"):
        ax.scatter(sub["aoi_ms"], sub["err_l2_mrad"],
                   s=6, alpha=0.4, c=[colours[profile]], label=profile)
    ax.set_xlabel("AoI (ms)")
    ax.set_ylabel("Sync error L2 (mrad)")
    ax.set_title("AoI vs synchronisation error")
    ax.grid(True, alpha=0.3)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.legend(loc="lower right", fontsize=8, markerscale=2)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


def print_summary(df: pd.DataFrame) -> None:
    print("\n=== Summary by network profile ===")
    df2 = df[df["network_profile"] != "unknown"]
    stats = (df2.groupby("network_profile")
                 .agg(rows=("aoi_s", "count"),
                      aoi_med_ms=("aoi_ms", "median"),
                      aoi_p95_ms=("aoi_ms", lambda x: x.quantile(0.95)),
                      err_med_mrad=("err_l2_mrad", "median"),
                      err_p95_mrad=("err_l2_mrad", lambda x: x.quantile(0.95)))
                 .round(2))
    print(stats.to_string())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="Path to aoi_logger CSV")
    parser.add_argument("--out-dir", type=Path, default=None,
                        help="Output directory (default: CSV's directory)")
    args = parser.parse_args()

    if not args.csv.is_file():
        print(f"CSV not found: {args.csv}", file=sys.stderr)
        return 1

    out_dir = args.out_dir or args.csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = args.csv.stem

    print(f"Loading {args.csv}...")
    df = load(args.csv)
    print(f"  {len(df)} rows, {df['t_rel'].iloc[-1]:.1f} s of data")
    print(f"  profiles seen: {sorted(df['network_profile'].unique())}")

    plot_aoi_timeline(df,    out_dir / f"{stem}_aoi_timeline.png")
    plot_err_timeline(df,    out_dir / f"{stem}_err_timeline.png")
    plot_err_by_profile(df,  out_dir / f"{stem}_err_by_profile.png")
    plot_aoi_vs_err(df,      out_dir / f"{stem}_aoi_vs_err.png")

    print_summary(df)
    return 0


if __name__ == "__main__":
    sys.exit(main())
