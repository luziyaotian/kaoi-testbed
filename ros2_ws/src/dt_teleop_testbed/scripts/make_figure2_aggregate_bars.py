#!/usr/bin/env python3
"""
make_figure2_aggregate_bars.py

Generates Figure 2 for the paper: per-profile bars for
  Left   = standard AoI P99 (ms)
  Centre = ULAoI E_tau (mean excess above tau, ms)
  Right  = K-AoI eef P95 (mm)

with 95% CI error bars across N=11 trials.

Source: either the aggregate_summary.json from aggregate_runs.py,
or recompute by reading all enriched E3 CSVs.

Outputs:
  figure2_aggregate_bars.pdf  (vector, for the paper)
  figure2_aggregate_bars.png  (raster, for slides / preview)

Usage (recommended -- if aggregate_summary.json exists):
    python3 make_figure2_aggregate_bars.py \\
        --summary ~/testbed_logs/E3_aggregated/aggregate_summary.json \\
        --out-dir ~/testbed_logs/E3_aggregated/

Usage (recompute from scratch):
    python3 make_figure2_aggregate_bars.py \\
        --csvs '~/testbed_logs/E3_*_enriched.csv' \\
        --out-dir ~/testbed_logs/E3_aggregated/ \\
        --threshold-ms 200
"""

from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


PROFILE_ORDER = ["ideal", "lan", "wifi_good", "wifi_congested",
                 "4g", "poor_4g", "satellite"]

PROFILE_LABELS = {
    "ideal": "ideal",
    "lan": "LAN",
    "wifi_good": "WiFi (good)",
    "wifi_congested": "WiFi (cong.)",
    "4g": "4G",
    "poor_4g": "4G (poor)",
    "satellite": "satellite",
}


# ----------------------------------------------------------------- #
# Source A: parse aggregate_summary.json
# ----------------------------------------------------------------- #

def load_from_summary_json(json_path: Path):
    """Read precomputed aggregate stats. Returns dict[profile] -> dict[metric]."""
    with open(json_path) as f:
        summary = json.load(f)

    out = {}
    for profile, payload in summary.items():
        m = payload.get("metrics", {})

        def _get(key):
            d = m.get(key)
            if not d:
                return None
            return {
                "mean": d["mean"],
                "ci95_lower": d["ci95_lower"],
                "ci95_upper": d["ci95_upper"],
            }

        out[profile] = {
            "aoi_p99_ms":     _get("aoi_p99_ms"),
            "excess_mean_ms": _get("excess_mean_ms"),
            "kaoi_p95_mm":    _kaoi_p95(_get, m),
        }
    return out


def _kaoi_p95(_get, m):
    """K-AoI may be stored under several names depending on aggregate_runs version."""
    for key in ("kaoi_p95_mm", "k_aoi_eef_max_p95_mm",
                "k_aoi_eef_max_p95_m", "kaoi_eef_p95_mm"):
        d = m.get(key)
        if d:
            mean = d["mean"]
            lo = d["ci95_lower"]
            hi = d["ci95_upper"]
            # Convert m -> mm if needed (heuristic: < 1 means metres)
            if mean < 1:
                mean *= 1000; lo *= 1000; hi *= 1000
            return {"mean": mean, "ci95_lower": lo, "ci95_upper": hi}
    return None


# ----------------------------------------------------------------- #
# Source B: recompute from enriched CSVs
# ----------------------------------------------------------------- #

def load_from_csvs(csv_paths, threshold_ms: float, transient_s: float = 2.0):
    """Compute per-trial-per-profile stats, then mean+CI across trials."""
    threshold_s = threshold_ms / 1000.0
    per_trial = {p: {"aoi_p99_ms": [], "excess_mean_ms": [],
                     "kaoi_p95_mm": []} for p in PROFILE_ORDER}

    for path in csv_paths:
        df = pd.read_csv(path)
        if "network_profile" not in df.columns:
            continue

        # drop transient
        if transient_s > 0 and "wall_time_s" in df.columns:
            df = df.sort_values("wall_time_s").reset_index(drop=True)
            block_id = (df["network_profile"]
                        != df["network_profile"].shift()).cumsum()
            block_start = df.groupby(block_id)["wall_time_s"].transform("min")
            df = df[(df["wall_time_s"] - block_start) >= transient_s]

        for profile, sub in df.groupby("network_profile"):
            if profile not in per_trial:
                continue
            aoi_s = np.abs(sub["aoi_s"].dropna().to_numpy())
            if len(aoi_s) == 0:
                continue

            per_trial[profile]["aoi_p99_ms"].append(
                float(np.quantile(aoi_s, 0.99) * 1000))

            excess = aoi_s[aoi_s > threshold_s] - threshold_s
            per_trial[profile]["excess_mean_ms"].append(
                float(np.mean(excess) * 1000) if len(excess) else 0.0)

            if "k_aoi_eef_max_m" in sub.columns:
                kaoi = sub["k_aoi_eef_max_m"].dropna().to_numpy() * 1000
                if len(kaoi):
                    per_trial[profile]["kaoi_p95_mm"].append(
                        float(np.quantile(kaoi, 0.95)))

    out = {}
    for profile, metrics in per_trial.items():
        out[profile] = {}
        for key, vals in metrics.items():
            if not vals:
                out[profile][key] = None
                continue
            arr = np.array(vals, dtype=np.float64)
            mean = float(arr.mean())
            n = len(arr)
            sem = float(arr.std(ddof=1) / np.sqrt(n)) if n > 1 else 0.0
            t = 2.228 if n == 11 else 2.262 if n == 10 else 2.0
            ci = t * sem
            out[profile][key] = {
                "mean": mean,
                "ci95_lower": mean - ci,
                "ci95_upper": mean + ci,
            }
    return out


# ----------------------------------------------------------------- #
# Plotting
# ----------------------------------------------------------------- #

def _extract(profile_data, profiles, metric):
    means, lows, highs = [], [], []
    for p in profiles:
        d = profile_data.get(p, {}).get(metric)
        if d is None:
            means.append(np.nan); lows.append(0.0); highs.append(0.0)
        else:
            means.append(d["mean"])
            lows.append(d["mean"] - d["ci95_lower"])
            highs.append(d["ci95_upper"] - d["mean"])
    return (np.array(means, dtype=np.float64),
            np.array(lows,  dtype=np.float64),
            np.array(highs, dtype=np.float64))


def plot(profile_data, out_dir: Path, threshold_ms: float = 200.0):
    profiles = [p for p in PROFILE_ORDER if p in profile_data]
    labels = [PROFILE_LABELS[p] for p in profiles]
    x = np.arange(len(profiles))

    # IEEE two-column figure: text width ~ 7.16 in.
    # Use 7 in wide x 2.4 in tall for a 1x3 panel.
    fig, axes = plt.subplots(1, 3, figsize=(7.0, 2.4), constrained_layout=True)

    panel_specs = [
        ("aoi_p99_ms",     "Standard AoI (P99)",
         "AoI P99 (ms)",            "#1f77b4"),
        ("excess_mean_ms", f"ULAoI $E_\\tau$ "
                           f"($\\tau={int(threshold_ms)}$ ms)",
         r"$\mathbb{E}[\Delta - \tau \mid \Delta > \tau]$ (ms)",
                                                 "#ff7f0e"),
        ("kaoi_p95_mm",    "K-AoI$_\\mathrm{eef}$ (P95)",
         "K-AoI P95 (mm)",          "#d62728"),
    ]

    for ax, (key, title, ylabel, color) in zip(axes, panel_specs):
        means, lows, highs = _extract(profile_data, profiles, key)
        ax.bar(x, means, yerr=[lows, highs], color=color,
               edgecolor="black", linewidth=0.5,
               error_kw={"ecolor": "black", "lw": 0.8, "capsize": 2.5,
                         "capthick": 0.8})
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=7)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_title(title, fontsize=9)
        ax.tick_params(axis="y", labelsize=7)
        ax.grid(axis="y", alpha=0.3, linewidth=0.5)
        ax.set_axisbelow(True)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        if np.nanmax(means + highs) > 0:
            ax.set_ylim(0, np.nanmax(means + highs) * 1.10)

    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = out_dir / "figure2_aggregate_bars.pdf"
    png_path = out_dir / "figure2_aggregate_bars.png"
    fig.savefig(pdf_path, bbox_inches="tight")
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved:")
    print(f"  {pdf_path}  (vector, for paper \\includegraphics)")
    print(f"  {png_path}  (raster, for slides / preview)")
    return pdf_path, png_path


# ----------------------------------------------------------------- #
# Main
# ----------------------------------------------------------------- #

def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--summary",
                     help="path to aggregate_summary.json")
    src.add_argument("--csvs",
                     help="glob pattern for enriched CSVs")
    ap.add_argument("--out-dir", default=".", type=Path,
                    help="output directory for figure files")
    ap.add_argument("--threshold-ms", type=float, default=200.0)
    ap.add_argument("--transient-s", type=float, default=2.0,
                    help="(only with --csvs) seconds to drop after "
                         "each profile change")
    args = ap.parse_args()

    if args.summary:
        path = Path(os.path.expanduser(args.summary))
        print(f"Loading {path}")
        data = load_from_summary_json(path)
    else:
        pattern = os.path.expanduser(args.csvs)
        files = sorted(glob.glob(pattern))
        if not files:
            raise SystemExit(f"No files match: {pattern}")
        print(f"Computing from {len(files)} CSVs:")
        for f in files:
            print(f"  {os.path.basename(f)}")
        data = load_from_csvs(files,
                              threshold_ms=args.threshold_ms,
                              transient_s=args.transient_s)

    plot(data, args.out_dir, threshold_ms=args.threshold_ms)


if __name__ == "__main__":
    main()
