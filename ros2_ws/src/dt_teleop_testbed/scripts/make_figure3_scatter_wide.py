#!/usr/bin/env python3
"""
make_figure3_scatter.py

Generates Figure 3 for the paper: 2-panel side-by-side scatter plots
of AoI (ms) vs K-AoI (mm) for the two extremal profiles (ideal and
satellite), pooled across all N=11 trials. Points coloured by twin
end-effector velocity. Spearman rank correlation reported in each
panel title.

This is the paper's central empirical evidence that K-AoI captures
information AoI alone cannot — the decoupling is most extreme at
the network extremes (ρ = 0.29 ideal, ρ = 0.22 satellite).

Usage:
    python3 make_figure3_scatter.py \\
        ~/testbed_logs/E3_*_enriched.csv \\
        --out-dir ~/testbed_logs/E3_aggregated/ \\
        --transient-s 2.0

Outputs:
    figure3_aoi_vs_kaoi_scatter.pdf   (vector, for paper)
    figure3_aoi_vs_kaoi_scatter.png   (raster, for preview / slides)
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from scipy.stats import spearmanr


# Profiles to plot, in panel order (left, right)
PROFILES_TO_PLOT = ["ideal", "satellite"]


def drop_transient(df: pd.DataFrame, transient_s: float) -> pd.DataFrame:
    """Drop the first `transient_s` seconds of each profile run."""
    if transient_s <= 0 or "wall_time_s" not in df.columns:
        return df
    df = df.sort_values("wall_time_s").reset_index(drop=True)
    block_id = (df["network_profile"] != df["network_profile"].shift()).cumsum()
    block_start = df.groupby(block_id)["wall_time_s"].transform("min")
    return df[(df["wall_time_s"] - block_start) >= transient_s].reset_index(drop=True)


def load_pooled(csv_paths, profile: str, transient_s: float):
    """Pool AoI / K-AoI / velocity samples across all trials for one profile."""
    aoi_chunks, kaoi_chunks, vel_chunks = [], [], []
    for path in csv_paths:
        try:
            df = pd.read_csv(path)
        except Exception as exc:
            print(f"  WARNING: skipping {path}: {exc}")
            continue
        df = drop_transient(df, transient_s)
        sub = df[df["network_profile"] == profile]
        if len(sub) == 0:
            continue
        aoi_ms = np.abs(sub["aoi_s"].to_numpy()) * 1000
        kaoi_mm = sub["k_aoi_eef_max_m"].to_numpy() * 1000
        if "twin_velocity_mps" in sub.columns:
            vel_mmps = sub["twin_velocity_mps"].to_numpy() * 1000
        else:
            vel_mmps = np.full_like(aoi_ms, np.nan)

        # Drop NaNs jointly
        mask = np.isfinite(aoi_ms) & np.isfinite(kaoi_mm) & np.isfinite(vel_mmps)
        aoi_chunks.append(aoi_ms[mask])
        kaoi_chunks.append(kaoi_mm[mask])
        vel_chunks.append(vel_mmps[mask])

    if not aoi_chunks:
        return np.array([]), np.array([]), np.array([])
    return (np.concatenate(aoi_chunks),
            np.concatenate(kaoi_chunks),
            np.concatenate(vel_chunks))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+", help="enriched CSV file paths")
    ap.add_argument("--out-dir", type=Path, default=Path("."),
                    help="output directory")
    ap.add_argument("--transient-s", type=float, default=2.0,
                    help="seconds to drop after each profile change")
    ap.add_argument("--max-points", type=int, default=8000,
                    help="downsample large pooled samples for "
                         "rendering speed (default 8000 per panel)")
    ap.add_argument("--seed", type=int, default=42,
                    help="random seed for downsampling")
    args = ap.parse_args()

    csv_paths = [os.path.expanduser(c) for c in args.csvs]
    print(f"Loading {len(csv_paths)} CSVs...")

    # Load both profiles
    panel_data = {}
    for profile in PROFILES_TO_PLOT:
        aoi, kaoi, vel = load_pooled(csv_paths, profile, args.transient_s)
        if len(aoi) == 0:
            print(f"  WARNING: no data for profile '{profile}'")
            continue

        # Compute Spearman BEFORE downsampling — we want the true ρ
        rho, p = spearmanr(aoi, kaoi)
        n_total = len(aoi)

        # Downsample if huge (purely visual; doesn't affect ρ shown)
        if len(aoi) > args.max_points:
            rng = np.random.default_rng(args.seed)
            idx = rng.choice(len(aoi), size=args.max_points, replace=False)
            aoi, kaoi, vel = aoi[idx], kaoi[idx], vel[idx]

        panel_data[profile] = {
            "aoi": aoi, "kaoi": kaoi, "vel": vel,
            "rho": float(rho), "p": float(p), "n": int(n_total),
        }
        print(f"  {profile:<12s}: n_pooled = {n_total:>6d}, "
              f"rho = {rho:+.3f}, p = {p:.2e}")

    if not panel_data:
        raise SystemExit("No data loaded for either profile.")

    # Shared colour scale across both panels for fair visual comparison
    all_vel = np.concatenate([d["vel"] for d in panel_data.values()])
    vmin = float(np.quantile(all_vel, 0.01))
    vmax = float(np.quantile(all_vel, 0.99))
    norm = Normalize(vmin=vmin, vmax=vmax)
    cmap = plt.cm.viridis

    # ---- Plot ----
    # IEEE two-column figure*: usable text width ~7.16 in.
    # Use 7.0 wide x 3.0 tall, leaving room for shared colour-bar.
    fig, axes = plt.subplots(1, len(panel_data),
                             figsize=(7.0, 3.0),
                             sharey=False)
    if len(panel_data) == 1:
        axes = [axes]

    for ax, profile in zip(axes, PROFILES_TO_PLOT):
        if profile not in panel_data:
            continue
        d = panel_data[profile]
        sc = ax.scatter(d["aoi"], d["kaoi"], c=d["vel"],
                        cmap=cmap, norm=norm,
                        s=4, alpha=0.55, linewidths=0,
                        rasterized=True)
        ax.set_xlabel("AoI (ms)", fontsize=9)
        ax.set_ylabel(r"K-AoI$_\mathrm{eef}$ (mm)", fontsize=9)
        # Title with ρ value (use n_total, not downsampled)
        ax.set_title(f"{profile}   "
                     f"$\\rho = {d['rho']:+.3f}$   "
                     f"$n = {d['n']:,}$",
                     fontsize=9)
        ax.grid(True, alpha=0.3, linewidth=0.4)
        ax.tick_params(axis="both", labelsize=8)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        # x-axis from 0 to slightly above max
        

    # Single shared colour-bar on the right
    fig.subplots_adjust(left=0.07, right=0.88, bottom=0.16, top=0.88,
                        wspace=0.30)
    cbar_ax = fig.add_axes([0.91, 0.16, 0.018, 0.72])
    cbar = fig.colorbar(sc, cax=cbar_ax)
    cbar.set_label("Twin velocity (mm/s)", fontsize=9)
    cbar.ax.tick_params(labelsize=8)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = args.out_dir / "figure3_aoi_vs_kaoi_scatter.pdf"
    png_path = args.out_dir / "figure3_aoi_vs_kaoi_scatter.png"
    fig.savefig(pdf_path, bbox_inches="tight", dpi=300)
    fig.savefig(png_path, bbox_inches="tight", dpi=200)
    plt.close(fig)

    print()
    print("Saved:")
    print(f"  {pdf_path}  (vector, for paper \\includegraphics)")
    print(f"  {png_path}  (raster, for slides / preview)")


if __name__ == "__main__":
    main()
