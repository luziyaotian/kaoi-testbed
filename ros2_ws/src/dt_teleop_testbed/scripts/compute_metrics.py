#!/usr/bin/env python3
"""
compute_metrics.py — enrich an aoi_logger CSV with derived metrics.

Given a CSV produced by aoi_logger_node (containing per-tick twin and real
joint positions, AoI, network profile), this script adds:

    - twin_x, twin_y, twin_z             : end-effector Cartesian position
    - real_x, real_y, real_z             : end-effector Cartesian position
    - cart_err_m                          : ||twin_xyz - real_xyz||  (metres)
    - twin_velocity_mps                   : finite-difference speed of twin EEF
    - real_velocity_mps                   : finite-difference speed of real EEF
    - k_aoi_eef_twin_m                    : twin_velocity * AoI (metres)
    - k_aoi_eef_real_m                    : real_velocity * AoI (metres)

Also computes ULAoI statistics per profile (bound-violation probability,
conditional mean and second moment of excess AoI) and writes a summary JSON.

Usage:
    python3 compute_metrics.py input.csv
    python3 compute_metrics.py input.csv --aoi-threshold-ms 100 --out-csv enriched.csv

Paper figure generation is separate: see plot_metrics.py.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np
import pandas as pd

# If run as a package, prefer the packaged module. Standalone fallback too.
try:
    from dt_teleop_testbed.lite6_kinematics import fk_positions
except ImportError:
    # Fallback for running from a checkout: scripts/ is sibling of pkg dir.
    _here = Path(__file__).resolve().parent
    sys.path.insert(0, str(_here.parent / "dt_teleop_testbed"))
    sys.path.insert(0, str(_here.parent))
    try:
        from dt_teleop_testbed.lite6_kinematics import fk_positions
    except ImportError:
        from lite6_kinematics import fk_positions


N_JOINTS = 6


def _load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = ["wall_time_s", "aoi_s", "network_profile"]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"Missing required column: {col}")
    # Validate joint columns
    for i in range(1, N_JOINTS + 1):
        for kind in ("twin", "real"):
            col = f"{kind}_joint{i}"
            if col not in df.columns:
                raise ValueError(f"Missing required column: {col}")
    # Drop rows where AoI is NaN (logger warm-up)
    df = df.dropna(subset=["aoi_s"]).reset_index(drop=True)
    df["t_rel"] = df["wall_time_s"] - df["wall_time_s"].iloc[0]
    return df


def _compute_xyz(df: pd.DataFrame, kind: str) -> np.ndarray:
    """Compute end-effector (x, y, z) for each row, given 6 joint columns."""
    angles = df[[f"{kind}_joint{i}" for i in range(1, 7)]].to_numpy()
    xyz = np.empty((len(angles), 3), dtype=np.float64)
    for i, q in enumerate(angles):
        if np.any(np.isnan(q)):
            xyz[i] = np.nan
        else:
            xyz[i] = fk_positions(q)
    return xyz


def _compute_velocity(xyz: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Finite-difference magnitude velocity (m/s). Pads first entry with 0."""
    dt = np.diff(t)
    with np.errstate(invalid="ignore"):
        dp = np.diff(xyz, axis=0)
        speeds = np.linalg.norm(dp, axis=1) / np.maximum(dt, 1e-9)
    # Smooth a tiny bit to avoid noise blow-ups
    if len(speeds) >= 5:
        w = 5
        kernel = np.ones(w) / w
        speeds_padded = np.concatenate([[speeds[0]] * (w // 2), speeds,
                                        [speeds[-1]] * (w // 2)])
        speeds = np.convolve(speeds_padded, kernel, mode="valid")
    return np.concatenate([[0.0], speeds])


# ============================================================== #
# ULAoI — following Liao et al. 2023
# ============================================================== #

def ulaoi_stats(aoi_s: np.ndarray, threshold_s: float) -> Dict[str, float]:
    """Compute the three ULAoI statistical constraints.

    Parameters
    ----------
    aoi_s : array of AoI samples (seconds).
    threshold_s : predefined bound zeta_max^AoI (seconds).

    Returns
    -------
    dict with:
        n_samples                    Total AoI samples considered
        p_violation                  Pr{AoI > threshold}
        n_excess                     Number of samples exceeding threshold
        excess_mean_s                E[AoI - threshold | AoI > threshold]
        excess_second_moment_s2      E[(AoI - threshold)^2 | ...]
        gpd_scale_hat                Generalised Pareto scale estimate
        gpd_shape_hat                Generalised Pareto shape estimate
        aoi_mean_s                   Mean of all AoI samples
        aoi_p95_s                    95th percentile
        aoi_p99_s                    99th percentile
    """
    aoi_s = np.asarray(aoi_s, dtype=np.float64)
    aoi_s = aoi_s[~np.isnan(aoi_s)]
    # Take absolute value — twin/real clocks may differ; the metric is timeliness.
    aoi_s = np.abs(aoi_s)

    n = len(aoi_s)
    if n == 0:
        return {k: float("nan") for k in
                ("p_violation", "excess_mean_s", "excess_second_moment_s2",
                 "gpd_scale_hat", "gpd_shape_hat",
                 "aoi_mean_s", "aoi_p95_s", "aoi_p99_s")} | {"n_samples": 0, "n_excess": 0}

    excess = aoi_s[aoi_s > threshold_s] - threshold_s
    n_excess = len(excess)
    p_viol = n_excess / n

    if n_excess >= 2:
        excess_mean = float(np.mean(excess))
        excess_sqm = float(np.mean(excess ** 2))
        # Method-of-moments GPD estimate (adequate for first-order stats).
        # See Hosking & Wallis 1987.
        var = float(np.var(excess))
        mean = excess_mean if excess_mean > 0 else 1e-9
        shape_hat = 0.5 * (1.0 - mean * mean / max(var, 1e-12))
        scale_hat = mean * (1.0 - shape_hat)
    else:
        excess_mean = excess_sqm = shape_hat = scale_hat = float("nan")

    return {
        "n_samples": int(n),
        "n_excess": int(n_excess),
        "p_violation": float(p_viol),
        "excess_mean_s": excess_mean,
        "excess_second_moment_s2": excess_sqm,
        "gpd_scale_hat": float(scale_hat) if not math.isnan(scale_hat) else float("nan"),
        "gpd_shape_hat": float(shape_hat) if not math.isnan(shape_hat) else float("nan"),
        "aoi_mean_s": float(np.mean(aoi_s)),
        "aoi_p95_s": float(np.quantile(aoi_s, 0.95)),
        "aoi_p99_s": float(np.quantile(aoi_s, 0.99)),
    }


# ============================================================== #
# Main
# ============================================================== #

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="Input aoi_logger CSV")
    parser.add_argument("--out-csv", type=Path, default=None,
                        help="Output enriched CSV (default: input_enriched.csv)")
    parser.add_argument("--out-summary", type=Path, default=None,
                        help="Output JSON summary (default: input_summary.json)")
    parser.add_argument("--aoi-threshold-ms", type=float, default=100.0,
                        help="ULAoI threshold zeta_max^AoI in milliseconds (default 100)")
    parser.add_argument("--profile-transient-s", type=float, default=2.0,
                        help="Drop this many seconds after each profile change")
    args = parser.parse_args()

    if not args.csv.is_file():
        print(f"[ERROR] CSV not found: {args.csv}", file=sys.stderr)
        return 1

    out_csv = args.out_csv or args.csv.with_name(args.csv.stem + "_enriched.csv")
    out_summary = args.out_summary or args.csv.with_name(args.csv.stem + "_summary.json")

    print(f"Loading {args.csv}")
    df = _load(args.csv)
    print(f"  {len(df)} rows, {df['t_rel'].iloc[-1]:.1f}s duration")
    print(f"  profiles: {sorted(df['network_profile'].unique())}")

    print("Computing forward kinematics...")
    twin_xyz = _compute_xyz(df, "twin")
    real_xyz = _compute_xyz(df, "real")

    df["twin_x"] = twin_xyz[:, 0]
    df["twin_y"] = twin_xyz[:, 1]
    df["twin_z"] = twin_xyz[:, 2]
    df["real_x"] = real_xyz[:, 0]
    df["real_y"] = real_xyz[:, 1]
    df["real_z"] = real_xyz[:, 2]
    df["cart_err_m"] = np.linalg.norm(twin_xyz - real_xyz, axis=1)

    print("Computing velocities...")
    t = df["wall_time_s"].to_numpy()
    df["twin_velocity_mps"] = _compute_velocity(twin_xyz, t)
    df["real_velocity_mps"] = _compute_velocity(real_xyz, t)

    print("Computing K-AoI...")
    aoi = np.abs(df["aoi_s"].to_numpy())    # use magnitude (handles neg clock drift)
    df["aoi_abs_s"] = aoi
    df["k_aoi_eef_twin_m"] = df["twin_velocity_mps"] * aoi
    df["k_aoi_eef_real_m"] = df["real_velocity_mps"] * aoi
    df["k_aoi_eef_max_m"] = df[["k_aoi_eef_twin_m", "k_aoi_eef_real_m"]].max(axis=1)

    print("Computing per-profile ULAoI statistics...")
    thresh_s = args.aoi_threshold_ms / 1000.0

    # Drop the first N seconds after each profile change (transient
    # period when the network conditioner is re-setting tc rules).
    keep_mask = np.ones(len(df), dtype=bool)
    current = df["network_profile"].iloc[0]
    start = df["t_rel"].iloc[0]
    for i in range(1, len(df)):
        if df["network_profile"].iloc[i] != current:
            current = df["network_profile"].iloc[i]
            start = df["t_rel"].iloc[i]
        if df["t_rel"].iloc[i] < start + args.profile_transient_s:
            keep_mask[i] = False
    df_stable = df[keep_mask]

    summary: Dict[str, dict] = {
        "input_csv": str(args.csv.resolve()),
        "aoi_threshold_s": thresh_s,
        "aoi_threshold_ms": args.aoi_threshold_ms,
        "profile_transient_s_dropped": args.profile_transient_s,
        "profiles": {},
        "overall_across_stable_data": {},
    }

    for profile, sub in df_stable.groupby("network_profile"):
        if profile == "unknown":
            continue
        stats = ulaoi_stats(sub["aoi_abs_s"].to_numpy(), thresh_s)
        # Append K-AoI stats too
        kaoi = sub["k_aoi_eef_max_m"].dropna().to_numpy() * 1000.0  # to mm
        cart_err_mm = sub["cart_err_m"].dropna().to_numpy() * 1000.0
        if len(kaoi):
            stats["kaoi_mean_mm"] = float(np.mean(kaoi))
            stats["kaoi_p95_mm"] = float(np.quantile(kaoi, 0.95))
            stats["kaoi_p99_mm"] = float(np.quantile(kaoi, 0.99))
        if len(cart_err_mm):
            stats["cart_err_mean_mm"] = float(np.mean(cart_err_mm))
            stats["cart_err_p95_mm"] = float(np.quantile(cart_err_mm, 0.95))
        summary["profiles"][profile] = stats

    summary["overall_across_stable_data"] = ulaoi_stats(
        df_stable["aoi_abs_s"].to_numpy(), thresh_s)

    print(f"Writing enriched CSV: {out_csv}")
    df.to_csv(out_csv, index=False)
    print(f"Writing summary JSON: {out_summary}")
    out_summary.write_text(json.dumps(summary, indent=2))

    # ---------------- Friendly console summary ---------------- #
    print("\n=== Per-profile summary ===")
    hdr = ["profile", "N", "AoI_mean", "AoI_p99", "Pr>τ", "E[excess]",
           "K-AoI_p95", "cart_err_p95"]
    print(" | ".join(f"{h:>12}" for h in hdr))
    for profile, s in summary["profiles"].items():
        row = [
            profile[:12],
            f"{s['n_samples']}",
            f"{s['aoi_mean_s']*1000:.1f} ms",
            f"{s['aoi_p99_s']*1000:.1f} ms",
            f"{s['p_violation']:.3f}",
            (f"{s['excess_mean_s']*1000:.1f} ms"
                if not math.isnan(s["excess_mean_s"]) else "--"),
            (f"{s.get('kaoi_p95_mm', float('nan')):.2f} mm"),
            (f"{s.get('cart_err_p95_mm', float('nan')):.2f} mm"),
        ]
        print(" | ".join(f"{c:>12}" for c in row))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
