#!/usr/bin/env python3
"""
aggregate_runs.py — combine multiple E3 sweep CSVs into statistics.

Takes N enriched CSVs (from compute_metrics.py), computes per-profile
mean ± 95% CI across runs, and produces:
    - combined_summary.json  with per-profile per-run per-metric values
    - aggregate_bars.png     bar chart of means with CI error bars

Usage:
    python3 aggregate_runs.py \
        ~/testbed_logs/E3_*_enriched.csv \
        --out-dir ~/testbed_logs/aggregated \
        --threshold-ms 200
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


IEEE_DOUBLE_WIDTH_IN = 7.0
CANONICAL_PROFILE_ORDER = [
    "ideal", "lan", "wifi_good", "wifi_congested", "4g", "poor_4g", "satellite",
]


def _profile_key(name: str) -> int:
    """Return sort key so profiles print in network-quality order."""
    try:
        return CANONICAL_PROFILE_ORDER.index(name)
    except ValueError:
        return 99


def compute_per_run_per_profile(df: pd.DataFrame,
                                threshold_s: float,
                                transient_s: float = 2.0) -> Dict[str, dict]:
    """
    Given one enriched CSV as a DataFrame, compute per-profile stats.
    Drops transient_s after each profile change (netem settling).
    """
    # Drop transient period after each profile change
    keep = np.ones(len(df), dtype=bool)
    if "t_rel" not in df.columns:
        df["t_rel"] = df["wall_time_s"] - df["wall_time_s"].iloc[0]
    current = df["network_profile"].iloc[0]
    start = df["t_rel"].iloc[0]
    for i in range(1, len(df)):
        if df["network_profile"].iloc[i] != current:
            current = df["network_profile"].iloc[i]
            start = df["t_rel"].iloc[i]
        if df["t_rel"].iloc[i] < start + transient_s:
            keep[i] = False
    stable = df[keep]

    result: Dict[str, dict] = {}
    for profile, sub in stable.groupby("network_profile"):
        if profile == "unknown":
            continue
        aoi = np.abs(sub["aoi_s"].dropna().to_numpy())
        kaoi = (sub["k_aoi_eef_max_m"].dropna().to_numpy() * 1000.0
                if "k_aoi_eef_max_m" in sub.columns else np.array([]))
        cart = (sub["cart_err_m"].dropna().to_numpy() * 1000.0
                if "cart_err_m" in sub.columns else np.array([]))
        excess = aoi[aoi > threshold_s] - threshold_s

        result[profile] = {
            "n_samples": int(len(aoi)),
            "aoi_mean_ms": float(np.mean(aoi) * 1000) if len(aoi) else np.nan,
            "aoi_p95_ms": float(np.quantile(aoi, 0.95) * 1000) if len(aoi) else np.nan,
            "aoi_p99_ms": float(np.quantile(aoi, 0.99) * 1000) if len(aoi) else np.nan,
            "p_violation": float(np.mean(aoi > threshold_s)) if len(aoi) else np.nan,
            "excess_mean_ms": float(np.mean(excess) * 1000) if len(excess) else 0.0,
            "kaoi_mean_mm": float(np.mean(kaoi)) if len(kaoi) else np.nan,
            "kaoi_p95_mm": float(np.quantile(kaoi, 0.95)) if len(kaoi) else np.nan,
            "kaoi_p99_mm": float(np.quantile(kaoi, 0.99)) if len(kaoi) else np.nan,
            "cart_err_p95_mm": float(np.quantile(cart, 0.95)) if len(cart) else np.nan,
        }
    return result


def aggregate(results_per_run: List[Dict[str, dict]]) -> Dict[str, dict]:
    """Mean ± 95% CI across runs for each profile and metric."""
    # Collect values
    profiles = sorted({p for run in results_per_run for p in run}, key=_profile_key)
    metrics = [
        "aoi_mean_ms", "aoi_p95_ms", "aoi_p99_ms", "p_violation",
        "excess_mean_ms", "kaoi_mean_mm", "kaoi_p95_mm", "kaoi_p99_mm",
        "cart_err_p95_mm",
    ]

    agg: Dict[str, dict] = {}
    for profile in profiles:
        agg[profile] = {"n_runs": 0, "metrics": {}}
        for metric in metrics:
            values = [run[profile][metric] for run in results_per_run
                      if profile in run and not np.isnan(run[profile][metric])]
            agg[profile]["n_runs"] = max(agg[profile]["n_runs"], len(values))
            if len(values) == 0:
                agg[profile]["metrics"][metric] = {
                    "mean": np.nan, "std": np.nan,
                    "ci95_lower": np.nan, "ci95_upper": np.nan, "n": 0}
            else:
                values = np.asarray(values, dtype=np.float64)
                mean = float(np.mean(values))
                std = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
                # 95% CI using t-distribution approx (for small n we'd want
                # actual t, but for n>=5 this is fine for visualization)
                sem = std / max(1.0, np.sqrt(len(values)))
                ci = 1.96 * sem
                agg[profile]["metrics"][metric] = {
                    "mean": mean, "std": std,
                    "ci95_lower": mean - ci, "ci95_upper": mean + ci,
                    "n": int(len(values)),
                    "values": values.tolist(),
                }
    return agg


def plot_agg_bars(agg: Dict[str, dict], out: Path) -> None:
    """3-panel bar chart: AoI_p99, ULAoI E[excess], K-AoI p95 — with CIs."""
    profiles = list(agg.keys())
    x = np.arange(len(profiles))

    fig, axes = plt.subplots(1, 3, figsize=(IEEE_DOUBLE_WIDTH_IN, 2.6))

    def _bar(ax, metric, colour, title, ylabel):
        means = [agg[p]["metrics"][metric]["mean"] for p in profiles]
        lowers = [agg[p]["metrics"][metric]["mean"] -
                  agg[p]["metrics"][metric]["ci95_lower"] for p in profiles]
        uppers = [agg[p]["metrics"][metric]["ci95_upper"] -
                  agg[p]["metrics"][metric]["mean"] for p in profiles]
        errs = np.array([lowers, uppers])
        ax.bar(x, means, color=colour, yerr=errs, capsize=3, ecolor="black")
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels(profiles, rotation=30, ha="right", fontsize=7)
        ax.grid(True, axis="y", alpha=0.3)

    _bar(axes[0], "aoi_p99_ms", "tab:blue",
         "Standard AoI (P99)", "ms")
    _bar(axes[1], "excess_mean_ms", "tab:orange",
         "ULAoI E[excess]", "ms")
    _bar(axes[2], "kaoi_p95_mm", "tab:red",
         "K-AoI (P95)", "mm")

    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  wrote {out}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csvs", type=Path, nargs="+",
                        help="Enriched CSVs from compute_metrics.py (one per run)")
    parser.add_argument("--threshold-ms", type=float, default=200.0)
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    if not args.csvs:
        print("No CSVs provided", file=sys.stderr)
        return 1
    out_dir = args.out_dir or args.csvs[0].parent / "aggregated"
    out_dir.mkdir(parents=True, exist_ok=True)

    threshold_s = args.threshold_ms / 1000.0

    print(f"Aggregating {len(args.csvs)} runs at threshold = {args.threshold_ms} ms")

    per_run_results = []
    for csv in args.csvs:
        if not csv.is_file():
            print(f"[WARN] missing: {csv}", file=sys.stderr)
            continue
        df = pd.read_csv(csv)
        print(f"  {csv.name}: {len(df)} rows")
        per_run_results.append(
            compute_per_run_per_profile(df, threshold_s))

    agg = aggregate(per_run_results)

    out_json = out_dir / "aggregate_summary.json"
    out_json.write_text(json.dumps(agg, indent=2))
    print(f"Wrote {out_json}")

    plot_agg_bars(agg, out_dir / "aggregate_bars.png")

    # Console summary
    print("\n=== Aggregated per-profile summary (mean ± CI95, N=runs) ===")
    print(f"{'profile':>14} | {'N':>3} | {'AoI P99 (ms)':>18} | "
          f"{'E[excess] (ms)':>18} | {'K-AoI P95 (mm)':>18}")
    for profile, d in agg.items():
        n = d["n_runs"]
        def fmt(m):
            v = d["metrics"][m]
            if np.isnan(v["mean"]):
                return "--"
            return f"{v['mean']:.1f} ± {v['mean'] - v['ci95_lower']:.1f}"
        print(f"{profile:>14} | {n:>3} | {fmt('aoi_p99_ms'):>18} | "
              f"{fmt('excess_mean_ms'):>18} | {fmt('kaoi_p95_mm'):>18}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
