#!/usr/bin/env python3
"""
make_figure3_scatter.py  (single-column variant)

Generates Figure 3 for the paper: 2-panel STACKED scatter plots
of AoI (ms) vs K-AoI (mm) for the two extremal profiles (ideal and
satellite), pooled across all N=11 trials. Points coloured by twin
end-effector velocity. Spearman rank correlation reported in each
panel title.

Single-column variant: 2 rows x 1 col, fits in \\columnwidth (~3.5 in).
Horizontal colour-bar at the bottom shared across both panels.

Usage:
    python3 make_figure3_scatter.py \\
        ~/testbed_logs/E3_*_enriched.csv \\
        --out-dir ~/testbed_logs/E3_aggregated/ \\
        --transient-s 2.0
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


PROFILES_TO_PLOT = ["ideal", "satellite"]


def drop_transient(df: pd.DataFrame, transient_s: float) -> pd.DataFrame:
    if transient_s <= 0 or "wall_time_s" not in df.columns:
        return df
    df = df.sort_values("wall_time_s").reset_index(drop=True)
    block_id = (df["network_profile"] != df["network_profile"].shift()).cumsum()
    block_start = df.groupby(block_id)["wall_time_s"].transform("min")
    return df[(df["wall_time_s"] - block_start) >= transient_s].reset_index(drop=True)


def load_pooled(csv_paths, profile: str, transient_s: float):
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
    ap.add_argument("--out-dir", type=Path, default=Path("."))
    ap.add_argument("--transient-s", type=float, default=2.0)
    ap.add_argument("--max-points", type=int, default=8000)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    csv_paths = [os.path.expanduser(c) for c in args.csvs]
    print(f"Loading {len(csv_paths)} CSVs...")

    panel_data = {}
    for profile in PROFILES_TO_PLOT:
        aoi, kaoi, vel = load_pooled(csv_paths, profile, args.transient_s)
        if len(aoi) == 0:
            print(f"  WARNING: no data for profile '{profile}'")
            continue

        # Compute Spearman BEFORE downsampling
        rho, p = spearmanr(aoi, kaoi)
        n_total = len(aoi)

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

    # Shared colour scale across panels
    all_vel = np.concatenate([d["vel"] for d in panel_data.values()])
    vmin = float(np.quantile(all_vel, 0.01))
    vmax = float(np.quantile(all_vel, 0.99))
    norm = Normalize(vmin=vmin, vmax=vmax)
    cmap = plt.cm.viridis

    # ---- Plot ----
    # Single-column IEEE: \columnwidth ~ 3.5 in. Two stacked panels +
    # horizontal colour-bar fits in ~5.2 in tall.
    fig, axes = plt.subplots(len(panel_data), 1,
                             figsize=(3.5, 5.2),
                             sharey=False)
    if len(panel_data) == 1:
        axes = [axes]

    for ax, profile in zip(axes, PROFILES_TO_PLOT):
        if profile not in panel_data:
            continue
        d = panel_data[profile]
        sc = ax.scatter(d["aoi"], d["kaoi"], c=d["vel"],
                        cmap=cmap, norm=norm,
                        s=3, alpha=0.55, linewidths=0,
                        rasterized=True)
        ax.set_xlabel("AoI (ms)", fontsize=8)
        ax.set_ylabel(r"K-AoI$_\mathrm{eef}$ (mm)", fontsize=8)
        ax.set_title(f"{profile}   "
                     f"$\\rho = {d['rho']:+.3f}$   "
                     f"$n = {d['n']:,}$",
                     fontsize=8)
        ax.grid(True, alpha=0.3, linewidth=0.4)
        ax.tick_params(axis="both", labelsize=7)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)

    # Layout: bottom margin must clear (a) bottom panel xtick labels,
    # (b) bottom panel xlabel, (c) gap, (d) colour-bar, (e) cbar label.
    fig.subplots_adjust(left=0.16, right=0.97, bottom=0.20, top=0.95,
                        hspace=0.55)
    cbar_ax = fig.add_axes([0.16, 0.06, 0.81, 0.018])
    cbar = fig.colorbar(sc, cax=cbar_ax, orientation="horizontal")
    cbar.set_label("Twin velocity (mm/s)", fontsize=8)
    cbar.ax.tick_params(labelsize=7)

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
