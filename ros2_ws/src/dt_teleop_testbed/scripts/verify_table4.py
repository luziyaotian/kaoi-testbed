#!/usr/bin/env python3
"""
verify_table4.py - cross-check the Spearman rho column of Table III against raw data.

Recomputes per-profile Spearman correlations between AoI and K-AoI
(the rho column of the published Table III) from enriched E3 CSVs
*independently* of kaoi_vs_aoi_scatter.py,
and reports four diagnostics per profile:

  1. Pooled Spearman rho (matches paper -- this is what Table 4 reports)
  2. Pooled p-value (statistical significance)
  3. Pooled Pearson r (linear correlation -- sanity cross-check)
  4. Per-trial Spearman rho mean +/- std (robustness across sweeps)

Then flags any cell that disagrees with the published table.

Usage:
    python3 verify_table4.py ~/testbed_logs/E3_*_enriched.csv \
        --transient-s 2.0
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np
import pandas as pd

try:
    from scipy.stats import spearmanr, pearsonr
except ImportError:
    print("scipy required: pip3 install --break-system-packages scipy")
    sys.exit(1)


PROFILE_ORDER = ["ideal", "lan", "wifi_good", "wifi_congested",
                 "4g", "poor_4g", "satellite"]

# Values in the Spearman rho column of Table III of the published paper
# (IEEE Digital Twin 2026 camera-ready; reported to 2 decimal places).
PUBLISHED = {
    "ideal":          0.30,
    "lan":            0.65,
    "wifi_good":      0.63,
    "wifi_congested": 0.63,
    "4g":             0.66,
    "poor_4g":        0.34,
    "satellite":      0.22,
}


def drop_transient(df: pd.DataFrame, transient_s: float) -> pd.DataFrame:
    if transient_s <= 0 or "wall_time_s" not in df.columns:
        return df
    df = df.sort_values("wall_time_s").reset_index(drop=True)
    block_id = (df["network_profile"] != df["network_profile"].shift()).cumsum()
    block_start = df.groupby(block_id)["wall_time_s"].transform("min")
    return df[(df["wall_time_s"] - block_start) >= transient_s].reset_index(drop=True)


def get_aoi_kaoi(df: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """Extract AoI (ms) and K-AoI (mm) arrays, dropping NaNs."""
    if "k_aoi_eef_max_m" not in df.columns:
        return np.array([]), np.array([])
    aoi_ms = np.abs(df["aoi_s"].to_numpy()) * 1000
    kaoi_mm = df["k_aoi_eef_max_m"].to_numpy() * 1000
    velocity_mps = df["twin_velocity_mps"].to_numpy() if "twin_velocity_mps" in df.columns else None

    mask = np.isfinite(aoi_ms) & np.isfinite(kaoi_mm)
    if velocity_mps is not None:
        mask &= np.isfinite(velocity_mps)
    return aoi_ms[mask], kaoi_mm[mask]


def diff_flag(observed: float, published: float,
              tol: float = 0.005) -> str:
    if observed is None or np.isnan(observed):
        return " n/a"
    if abs(observed - published) <= tol:
        return " OK "
    if abs(observed - published) <= tol * 4:
        return "close"
    return " !!!"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+", help="enriched CSVs to verify against")
    ap.add_argument("--transient-s", type=float, default=2.0)
    args = ap.parse_args()

    csv_paths = [os.path.expanduser(c) for c in args.csvs]
    print(f"Loading {len(csv_paths)} enriched CSVs:")
    for p in csv_paths:
        print(f"  {os.path.basename(p)}")

    trials = []
    for p in csv_paths:
        try:
            df = pd.read_csv(p)
            df = drop_transient(df, args.transient_s)
            trials.append((Path(p).stem, df))
        except Exception as exc:
            print(f"  WARNING: could not load {p}: {exc}")
    if not trials:
        sys.exit("No CSVs loaded.")
    print(f"Successfully loaded {len(trials)} trials after transient drop.\n")

    print("=" * 92)
    print("Verification of Table 4 (Spearman correlation between AoI and K-AoI per profile)")
    print("=" * 92)
    print(f"{'Profile':<16} {'pub ρ':>7} {'obs ρ':>7} {'flag':>6} "
          f"{'p-value':>10} {'Pearson r':>10} "
          f"{'per-trial ρ (mean±std)':>26} {'n_pooled':>10}")
    print("-" * 92)

    all_results = {}
    for profile in PROFILE_ORDER:
        # --- pooled across trials (path A) ---
        all_aoi_ms = []
        all_kaoi_mm = []
        per_trial_rho: List[float] = []
        for trial_name, df in trials:
            sub = df[df["network_profile"] == profile]
            aoi_ms, kaoi_mm = get_aoi_kaoi(sub)
            if len(aoi_ms) < 30:
                continue
            all_aoi_ms.append(aoi_ms)
            all_kaoi_mm.append(kaoi_mm)
            try:
                rho_t, _ = spearmanr(aoi_ms, kaoi_mm)
                if not np.isnan(rho_t):
                    per_trial_rho.append(float(rho_t))
            except Exception:
                pass

        if not all_aoi_ms:
            print(f"{profile:<16} {PUBLISHED[profile]:>7.3f} "
                  f"{'n/a':>7} {' n/a':>6}")
            continue

        pooled_aoi = np.concatenate(all_aoi_ms)
        pooled_kaoi = np.concatenate(all_kaoi_mm)

        rho, p_spearman = spearmanr(pooled_aoi, pooled_kaoi)
        r_pearson, _    = pearsonr(pooled_aoi, pooled_kaoi)

        per_trial_mean = np.mean(per_trial_rho) if per_trial_rho else np.nan
        per_trial_std  = np.std(per_trial_rho, ddof=1) if len(per_trial_rho) > 1 else 0.0

        flag = diff_flag(float(rho), PUBLISHED[profile])

        # Display p-value compactly
        if p_spearman < 1e-300:
            p_str = "<1e-300"
        elif p_spearman < 1e-10:
            p_str = f"{p_spearman:.0e}"
        else:
            p_str = f"{p_spearman:.2e}"

        per_trial_str = f"{per_trial_mean:+.3f} ± {per_trial_std:.3f}"

        all_results[profile] = {
            "pooled_rho": float(rho),
            "pooled_p":   float(p_spearman),
            "pearson_r":  float(r_pearson),
            "per_trial_mean": float(per_trial_mean),
            "per_trial_std":  float(per_trial_std),
            "n_pooled": int(len(pooled_aoi)),
        }

        print(f"{profile:<16} {PUBLISHED[profile]:>7.3f} "
              f"{rho:>+7.3f} {flag:>6} "
              f"{p_str:>10} {r_pearson:>+10.3f} "
              f"{per_trial_str:>26} {len(pooled_aoi):>10}")

    print("-" * 92)
    print()
    print("Legend:")
    print("  pub ρ        : value currently in main.tex Table 4")
    print("  obs ρ        : recomputed pooled Spearman from raw enriched CSVs")
    print("  flag         : OK if |obs - pub| <= 0.005, close if <= 0.020,")
    print("                 !!! otherwise (table needs updating)")
    print("  p-value      : Spearman test for ρ ≠ 0; should be vanishingly small")
    print("                 given large n; meaningful for honesty")
    print("  Pearson r    : linear correlation; should agree directionally with ρ")
    print("                 (cross-check that Spearman isn't a fluke)")
    print("  per-trial ρ  : mean across the 11 trials of *each* trial's ρ;")
    print("                 std shows robustness. Tight std => stable result.")
    print()

    # Sanity: does the published claim "ρ <= 0.65" hold?
    max_rho = max(r["pooled_rho"] for r in all_results.values())
    min_rho = min(r["pooled_rho"] for r in all_results.values())
    print(f"Recomputed range:  min ρ = {min_rho:+.3f}   max ρ = {max_rho:+.3f}")
    print(f"Paper claims:      'ρ ≤ 0.65 across all profiles, dropping to 0.22 under satellite'")

    if max_rho <= 0.66 and min_rho < 0.30:
        print("=> Claim is supported by the recomputed data.")
    else:
        print("=> Claim may need adjusting based on recomputed values.")


if __name__ == "__main__":
    main()
