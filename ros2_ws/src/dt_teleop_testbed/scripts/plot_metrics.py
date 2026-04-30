#!/usr/bin/env python3
"""
plot_metrics.py — paper figures from an enriched metrics CSV.

Generates four publication-ready figures comparing standard AoI, ULAoI,
and K-AoI under different network profiles:

    1. metrics_timeline.png        Time-series of AoI, K-AoI, velocity
    2. ccdf_aoi_by_profile.png     CCDF plot (tail of AoI per profile)
    3. kaoi_vs_carterr.png         Scatter: K-AoI prediction vs true error
    4. perprofile_comparison.png   Bar chart comparing all three metrics

Usage:
    python3 plot_metrics.py input_enriched.csv [--summary input_summary.json]

All figures honour IEEE single-column width conventions (3.5 inches).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


IEEE_SINGLE_WIDTH_IN = 3.5
IEEE_DOUBLE_WIDTH_IN = 7.0


def _load(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    # Ensure relative time column exists
    if "t_rel" not in df.columns:
        df["t_rel"] = df["wall_time_s"] - df["wall_time_s"].iloc[0]
    return df


def _profile_palette(profiles):
    cmap = plt.cm.tab10.colors
    return {p: cmap[i % len(cmap)] for i, p in enumerate(sorted(profiles))}


# ---------------------------------------------------------------- #
# Figure 1 — timeline
# ---------------------------------------------------------------- #

def plot_timeline(df: pd.DataFrame, out: Path) -> None:
    """Four-panel timeline: AoI, K-AoI, velocity, Cartesian error."""
    fig, axes = plt.subplots(4, 1, figsize=(IEEE_DOUBLE_WIDTH_IN, 6.5),
                             sharex=True)

    # Shade profile regions
    profiles = df["network_profile"].to_numpy()
    changes = [0]
    for i in range(1, len(profiles)):
        if profiles[i] != profiles[i - 1]:
            changes.append(i)
    changes.append(len(profiles))
    colours = _profile_palette(np.unique(profiles))

    for a in axes:
        for j in range(len(changes) - 1):
            start = df["t_rel"].iloc[changes[j]]
            end = df["t_rel"].iloc[changes[j + 1] - 1]
            name = profiles[changes[j]]
            a.axvspan(start, end, color=colours[name], alpha=0.12)

    # Annotate each span with the profile name on the top plot
    for j in range(len(changes) - 1):
        start = df["t_rel"].iloc[changes[j]]
        end = df["t_rel"].iloc[changes[j + 1] - 1]
        name = profiles[changes[j]]
        mid = 0.5 * (start + end)
        axes[0].annotate(name, xy=(mid, 0.95), xycoords=("data", "axes fraction"),
                         ha="center", va="top", fontsize=7, alpha=0.7,
                         rotation=0)

    axes[0].plot(df["t_rel"], df["aoi_abs_s"] * 1000, lw=0.5, color="k")
    axes[0].set_ylabel("AoI (ms)")
    axes[0].set_title("Standard AoI, velocity, K-AoI, and Cartesian error over time")

    axes[1].plot(df["t_rel"], df["twin_velocity_mps"] * 1000, lw=0.5,
                 color="tab:blue", label="twin")
    axes[1].plot(df["t_rel"], df["real_velocity_mps"] * 1000, lw=0.5,
                 color="tab:orange", label="real")
    axes[1].set_ylabel("v (mm/s)")
    axes[1].legend(loc="upper right", fontsize=7)

    axes[2].plot(df["t_rel"], df["k_aoi_eef_max_m"] * 1000, lw=0.5,
                 color="tab:red")
    axes[2].set_ylabel("K-AoI (mm)")

    axes[3].plot(df["t_rel"], df["cart_err_m"] * 1000, lw=0.5,
                 color="tab:purple")
    axes[3].set_ylabel("Cart. err (mm)")
    axes[3].set_xlabel("Time (s)")

    for a in axes:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


# ---------------------------------------------------------------- #
# Figure 2 — CCDF (ULAoI headline plot)
# ---------------------------------------------------------------- #

def plot_ccdf(df: pd.DataFrame, out: Path, threshold_ms: float) -> None:
    """CCDF of AoI per profile — shows the heavy tail that ULAoI captures."""
    fig, ax = plt.subplots(figsize=(IEEE_SINGLE_WIDTH_IN, IEEE_SINGLE_WIDTH_IN))

    profiles = sorted(p for p in df["network_profile"].unique() if p != "unknown")
    colours = _profile_palette(profiles)

    for profile in profiles:
        sub = df[df["network_profile"] == profile]
        aoi_ms = np.abs(sub["aoi_abs_s"].dropna().to_numpy()) * 1000
        if len(aoi_ms) == 0:
            continue
        aoi_sorted = np.sort(aoi_ms)
        ccdf = 1.0 - np.arange(len(aoi_sorted)) / len(aoi_sorted)
        ax.plot(aoi_sorted, ccdf, label=profile, color=colours[profile], lw=1.0)

    ax.axvline(threshold_ms, color="grey", ls=":", lw=1.0,
               label=f"τ = {threshold_ms:.0f} ms")
    ax.set_xlabel("AoI (ms)")
    ax.set_ylabel("CCDF: Pr(AoI > x)")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="lower left", fontsize=6)
    ax.set_title("AoI tail per network profile (log-log)")
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


# ---------------------------------------------------------------- #
# Figure 3 — K-AoI vs true Cartesian error (validation)
# ---------------------------------------------------------------- #

def plot_kaoi_validation(df: pd.DataFrame, out: Path) -> None:
    """Scatter: does K-AoI predict actual spatial divergence?"""
    fig, ax = plt.subplots(figsize=(IEEE_SINGLE_WIDTH_IN, IEEE_SINGLE_WIDTH_IN))

    df2 = df.dropna(subset=["k_aoi_eef_max_m", "cart_err_m"])
    kaoi_mm = df2["k_aoi_eef_max_m"].to_numpy() * 1000
    err_mm = df2["cart_err_m"].to_numpy() * 1000

    profiles = sorted(p for p in df2["network_profile"].unique() if p != "unknown")
    colours = _profile_palette(profiles)
    for profile in profiles:
        sub = df2[df2["network_profile"] == profile]
        k = sub["k_aoi_eef_max_m"].to_numpy() * 1000
        e = sub["cart_err_m"].to_numpy() * 1000
        ax.scatter(k, e, s=4, alpha=0.4, c=[colours[profile]], label=profile)

    # Identity line: perfect prediction
    lim = max(float(np.max(kaoi_mm)) if len(kaoi_mm) else 1,
              float(np.max(err_mm)) if len(err_mm) else 1, 1.0)
    ax.plot([0, lim], [0, lim], "k--", lw=0.8, label="y = x (identity)")

    # R² for informational display
    if len(kaoi_mm) > 1 and np.std(kaoi_mm) > 1e-9:
        r = float(np.corrcoef(kaoi_mm, err_mm)[0, 1])
        ax.text(0.03, 0.97, f"r = {r:.3f}", transform=ax.transAxes,
                fontsize=7, va="top")

    ax.set_xlabel("K-AoI prediction (mm)")
    ax.set_ylabel("Actual cart. error (mm)")
    ax.set_title("K-AoI vs measured spatial divergence")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=6, markerscale=2)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


# ---------------------------------------------------------------- #
# Figure 4 — per-profile comparison bars
# ---------------------------------------------------------------- #

def plot_perprofile(df: pd.DataFrame, out: Path, summary: dict) -> None:
    """Triple bar chart: standard AoI, ULAoI, K-AoI side-by-side per profile."""
    profiles_dict = summary.get("profiles", {})
    profiles = list(profiles_dict.keys())
    if not profiles:
        print(f"  skipping {out}: no profiles in summary")
        return

    aoi_p99  = [profiles_dict[p].get("aoi_p99_s", np.nan) * 1000 for p in profiles]
    excess   = [profiles_dict[p].get("excess_mean_s", np.nan) * 1000
                for p in profiles]
    kaoi_p95 = [profiles_dict[p].get("kaoi_p95_mm", np.nan) for p in profiles]

    fig, axes = plt.subplots(1, 3, figsize=(IEEE_DOUBLE_WIDTH_IN, 2.3))
    x = np.arange(len(profiles))

    axes[0].bar(x, aoi_p99, color="tab:blue")
    axes[0].set_title("Standard AoI (P99, ms)")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(profiles, rotation=30, ha="right", fontsize=7)
    axes[0].grid(True, axis="y", alpha=0.3)

    axes[1].bar(x, excess, color="tab:orange")
    axes[1].set_title("ULAoI E[excess] (ms)")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(profiles, rotation=30, ha="right", fontsize=7)
    axes[1].grid(True, axis="y", alpha=0.3)

    axes[2].bar(x, kaoi_p95, color="tab:red")
    axes[2].set_title("K-AoI (P95, mm)")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(profiles, rotation=30, ha="right", fontsize=7)
    axes[2].grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path,
                        help="Enriched CSV from compute_metrics.py")
    parser.add_argument("--summary", type=Path, default=None,
                        help="Summary JSON (default: inferred from csv stem)")
    parser.add_argument("--threshold-ms", type=float, default=100.0,
                        help="ULAoI threshold for CCDF annotation")
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    if not args.csv.is_file():
        print(f"[ERROR] CSV not found: {args.csv}", file=sys.stderr)
        return 1

    if args.summary is None:
        # Infer from stem: input_enriched.csv -> input_summary.json
        stem = args.csv.stem.replace("_enriched", "")
        args.summary = args.csv.with_name(stem + "_summary.json")

    if not args.summary.is_file():
        print(f"[WARN] Summary JSON not found at {args.summary}; "
              f"per-profile bar chart will be skipped.")
        summary = {"profiles": {}}
    else:
        summary = json.loads(args.summary.read_text())

    out_dir = args.out_dir or args.csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = args.csv.stem

    print(f"Loading {args.csv}")
    df = _load(args.csv)
    print(f"  {len(df)} rows")

    print("Plotting...")
    plot_timeline(df, out_dir / f"{stem}_timeline.png")
    plot_ccdf(df, out_dir / f"{stem}_ccdf.png", args.threshold_ms)
    plot_kaoi_validation(df, out_dir / f"{stem}_kaoi_validation.png")
    plot_perprofile(df, out_dir / f"{stem}_perprofile.png", summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
