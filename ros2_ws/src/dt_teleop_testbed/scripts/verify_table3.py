#!/usr/bin/env python3
"""
verify_table3.py — cross-check Table 3 of main.tex against raw data.

Recomputes per-profile statistics (AoI P99, P_tau, E_tau, K-AoI P95)
from enriched E3 CSVs *independently of aggregate_runs.py*, then
compares cell-by-cell with the values currently published in
the LaTeX table. Flags any discrepancy.

Two computation paths are reported:

  (A) Pooled across trials: pool all 11 sweeps' samples per profile
      and compute the percentiles / mean directly. This is what
      ULAoI papers typically report.

  (B) Per-trial then averaged: compute each statistic on each trial
      separately, then take the mean across the 11 trials.
      This is what "N=11 trials, mean +/- CI" implies.

The two CAN differ -- e.g., a heavy-tailed distribution's pooled P99
won't equal the average of per-trial P99s. Reviewers may ask which
is reported. We report both so you can pick the matching one.

Usage:
    python3 verify_table3.py ~/testbed_logs/E3_*_enriched.csv \
        --threshold-ms 200 --transient-s 2.0
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd


PROFILE_ORDER = ["ideal", "lan", "wifi_good", "wifi_congested",
                 "4g", "poor_4g", "satellite"]

# Values currently in main.tex Table 3 (tab:headline).
PUBLISHED = {
    "ideal":          {"aoi_p99": 199,  "p_tau": 0.7,  "e_tau": 1,   "kaoi_p95": 30},
    "lan":            {"aoi_p99": 200,  "p_tau": 1.3,  "e_tau": 1,   "kaoi_p95": 50},
    "wifi_good":      {"aoi_p99": 210,  "p_tau": 5.5,  "e_tau": 6,   "kaoi_p95": 50},
    "wifi_congested": {"aoi_p99": 280,  "p_tau": 22.3, "e_tau": 36,  "kaoi_p95": 75},
    "4g":             {"aoi_p99": 270,  "p_tau": 24.8, "e_tau": 36,  "kaoi_p95": 65},
    "poor_4g":        {"aoi_p99": 620,  "p_tau": 53.2, "e_tau": 129, "kaoi_p95": 130},
    "satellite":      {"aoi_p99": 1450, "p_tau": 80.6, "e_tau": 454, "kaoi_p95": 390},
}


def drop_transient(df: pd.DataFrame, transient_s: float) -> pd.DataFrame:
    """Drop the first `transient_s` seconds of each profile run."""
    if transient_s <= 0 or "wall_time_s" not in df.columns:
        return df
    df = df.sort_values("wall_time_s").reset_index(drop=True)
    block_id = (df["network_profile"] != df["network_profile"].shift()).cumsum()
    block_start = df.groupby(block_id)["wall_time_s"].transform("min")
    return df[(df["wall_time_s"] - block_start) >= transient_s].reset_index(drop=True)


def per_trial_stats(df_trial: pd.DataFrame, profile: str,
                    threshold_s: float) -> dict:
    """Compute the four metrics for one (trial, profile) slice."""
    sub = df_trial[df_trial["network_profile"] == profile]
    aoi_s = np.abs(sub["aoi_s"].dropna().to_numpy())

    out = {"aoi_p99": np.nan, "p_tau": np.nan,
           "e_tau": np.nan, "kaoi_p95": np.nan,
           "n": int(len(aoi_s))}
    if len(aoi_s) == 0:
        return out

    out["aoi_p99"] = float(np.quantile(aoi_s, 0.99) * 1000)

    excess = aoi_s[aoi_s > threshold_s] - threshold_s
    out["p_tau"] = float(len(excess) / len(aoi_s) * 100)
    out["e_tau"] = float(np.mean(excess) * 1000) if len(excess) else 0.0

    if "k_aoi_eef_max_m" in sub.columns:
        kaoi = sub["k_aoi_eef_max_m"].dropna().to_numpy() * 1000
        if len(kaoi):
            out["kaoi_p95"] = float(np.quantile(kaoi, 0.95))
    return out


def pooled_stats(all_aoi_s: np.ndarray, all_kaoi_mm: np.ndarray,
                 threshold_s: float) -> dict:
    """Compute the four metrics from pooled samples across trials."""
    if len(all_aoi_s) == 0:
        return {"aoi_p99": np.nan, "p_tau": np.nan,
                "e_tau": np.nan, "kaoi_p95": np.nan, "n": 0}
    excess = all_aoi_s[all_aoi_s > threshold_s] - threshold_s
    return {
        "n": int(len(all_aoi_s)),
        "aoi_p99": float(np.quantile(all_aoi_s, 0.99) * 1000),
        "p_tau":   float(len(excess) / len(all_aoi_s) * 100),
        "e_tau":   float(np.mean(excess) * 1000) if len(excess) else 0.0,
        "kaoi_p95": (float(np.quantile(all_kaoi_mm, 0.95))
                     if len(all_kaoi_mm) else np.nan),
    }


def fmt(value: float, kind: str) -> str:
    if value is None or np.isnan(value):
        return "  n/a"
    if kind == "aoi_p99":  return f"{value:7.1f}"
    if kind == "p_tau":    return f"{value:5.2f}%"
    if kind == "e_tau":    return f"{value:6.1f}"
    if kind == "kaoi_p95": return f"{value:6.1f}"
    return str(value)


def diff_flag(observed: float, published: float, kind: str) -> str:
    """Return ' OK', ' close' or '!!!' depending on relative deviation."""
    if observed is None or np.isnan(observed):
        return "n/a"
    # Tolerance per metric type: published table has limited precision.
    abs_tol = {"aoi_p99": 5, "p_tau": 0.5, "e_tau": 3, "kaoi_p95": 5}
    rel_tol = 0.10  # 10% relative
    abs_d = abs(observed - published)
    if abs_d <= abs_tol[kind]:
        return " OK "
    if abs_d <= max(abs_tol[kind] * 2, abs(published) * rel_tol):
        return "close"
    return " !!!"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+", help="enriched CSVs to verify against")
    ap.add_argument("--threshold-ms", type=float, default=200.0)
    ap.add_argument("--transient-s", type=float, default=2.0)
    args = ap.parse_args()

    threshold_s = args.threshold_ms / 1000.0
    csv_paths = [os.path.expanduser(c) for c in args.csvs]
    print(f"Loading {len(csv_paths)} enriched CSVs:")
    for p in csv_paths:
        print(f"  {os.path.basename(p)}")

    # Load all
    trials = []
    for p in csv_paths:
        try:
            df = pd.read_csv(p)
            df = drop_transient(df, args.transient_s)
            trials.append(df)
        except Exception as exc:
            print(f"  WARNING: could not load {p}: {exc}")
    if not trials:
        sys.exit("No CSVs loaded.")
    print(f"Successfully loaded {len(trials)} trials.\n")

    # =========== Per-trial then averaged (path B) ===========
    print("=" * 90)
    print("PATH B: per-trial statistics, averaged across N trials")
    print("=" * 90)
    print(f"{'Profile':<16} {'AoI P99 (ms)':<28} {'P_tau (%)':<22} "
          f"{'E_tau (ms)':<22} {'K-AoI P95 (mm)':<22}")
    print(f"{'':<16} {'pub | obs |  ' :<28}{'pub | obs |  ' :<22}"
          f"{'pub | obs |  ' :<22}{'pub | obs |  ' :<22}")
    print("-" * 110)

    avg_results = {}
    for profile in PROFILE_ORDER:
        per_trial = []
        for df in trials:
            per_trial.append(per_trial_stats(df, profile, threshold_s))
        # Average the per-trial scalars
        means = {k: np.nanmean([t[k] for t in per_trial])
                 for k in ("aoi_p99", "p_tau", "e_tau", "kaoi_p95")}
        avg_results[profile] = means

        pub = PUBLISHED[profile]
        line = f"{profile:<16}"
        for kind in ("aoi_p99", "p_tau", "e_tau", "kaoi_p95"):
            obs = means[kind]
            flag = diff_flag(obs, pub[kind], kind)
            line += f" {pub[kind]:>4} | {fmt(obs, kind)} | {flag}    "
        print(line)

    # =========== Pooled across trials (path A) ===========
    print()
    print("=" * 90)
    print("PATH A: pooled samples across all trials")
    print("=" * 90)
    print(f"{'Profile':<16} {'AoI P99 (ms)':<28} {'P_tau (%)':<22} "
          f"{'E_tau (ms)':<22} {'K-AoI P95 (mm)':<22}")
    print("-" * 110)

    pooled_results = {}
    for profile in PROFILE_ORDER:
        all_aoi = []
        all_kaoi = []
        for df in trials:
            sub = df[df["network_profile"] == profile]
            all_aoi.append(np.abs(sub["aoi_s"].dropna().to_numpy()))
            if "k_aoi_eef_max_m" in sub.columns:
                all_kaoi.append(sub["k_aoi_eef_max_m"].dropna().to_numpy() * 1000)
        all_aoi  = np.concatenate(all_aoi) if all_aoi else np.array([])
        all_kaoi = np.concatenate(all_kaoi) if all_kaoi else np.array([])
        stats = pooled_stats(all_aoi, all_kaoi, threshold_s)
        pooled_results[profile] = stats

        pub = PUBLISHED[profile]
        line = f"{profile:<16}"
        for kind in ("aoi_p99", "p_tau", "e_tau", "kaoi_p95"):
            obs = stats[kind]
            flag = diff_flag(obs, pub[kind], kind)
            line += f" {pub[kind]:>4} | {fmt(obs, kind)} | {flag}    "
        print(line)

    # =========== Summary ===========
    print()
    print("=" * 90)
    print("Legend:  OK = within absolute tolerance     "
          "close = within 10% rel    !!! = check this cell")
    print()
    print("If many cells say !!!, the table needs updating to match the data.")
    print("If pooled and per-trial-mean disagree heavily, the paper should")
    print("explicitly state which one it reports (or both).")


if __name__ == "__main__":
    main()
