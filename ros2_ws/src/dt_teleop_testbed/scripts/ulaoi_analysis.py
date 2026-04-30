#!/usr/bin/env python3
"""
ulaoi_analysis.py — rigorous ULAoI analysis following Liao et al. 2023.

Provides three improvements over the basic MoM GPD estimator:

  1. MLE GPD fit via scipy.stats.genpareto.fit (much better for heavy tails
     than method-of-moments; MoM fails when shape > ~0.25).
  2. KS goodness-of-fit test: does the observed tail actually follow a
     GPD? If p > 0.05 we can report ULAoI stats; if p < 0.05 we must
     flag that the tail is NOT GPD-distributed (caveat in paper).
  3. Mean Residual Life plot for threshold selection (peaks-over-threshold
     theory requires τ high enough that tail -> GPD).

Also runs on synthetic data matching the 7 network profiles, so you can
validate the pipeline before trusting it on real E3 data.

Usage:
    # On real enriched CSV:
    python3 ulaoi_analysis.py my_enriched.csv --threshold-ms 200

    # Synthetic validation run (no CSV needed):
    python3 ulaoi_analysis.py --synthetic

Output:
    Prints summary table per profile, including KS p-value, fit quality,
    and comparison MoM vs MLE. Saves mrl_plot_<profile>.png if requested.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

try:
    from scipy.stats import genpareto, kstest
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    import matplotlib
    matplotlib.use("Agg")  # headless-safe
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


# --------------------------------------------------------------------
# GPD fitting
# --------------------------------------------------------------------

def fit_gpd_mom(excess: np.ndarray) -> Tuple[float, float]:
    """Method-of-moments GPD fit. Cheap, biased for heavy tails.

    Returns (shape, scale). Shape is the GPD xi parameter; scale is sigma.
    """
    n = len(excess)
    if n < 2:
        return float("nan"), float("nan")
    mean = float(np.mean(excess))
    var = float(np.var(excess))
    if mean <= 0 or var <= 0:
        return float("nan"), float("nan")
    shape = 0.5 * (1.0 - mean * mean / var)
    scale = mean * (1.0 - shape)
    return shape, scale


def fit_gpd_mle(excess: np.ndarray) -> Tuple[float, float, Optional[str]]:
    """MLE GPD fit via scipy. Preferred for heavy-tailed data.

    Returns (shape, scale, warning_or_none).
    """
    if not HAS_SCIPY:
        return float("nan"), float("nan"), "scipy not installed"
    if len(excess) < 10:
        return float("nan"), float("nan"), f"only {len(excess)} excesses"
    try:
        # scipy's genpareto uses (c, loc, scale); we fix loc=0 since we
        # already subtracted the threshold.
        shape, _, scale = genpareto.fit(excess, floc=0)
        return float(shape), float(scale), None
    except Exception as exc:
        return float("nan"), float("nan"), f"fit failed: {exc}"


def gpd_ks_test(excess: np.ndarray, shape: float,
                scale: float) -> Tuple[float, float]:
    """Kolmogorov-Smirnov goodness-of-fit test against fitted GPD.

    Returns (KS_statistic, p_value). Small p-value (<0.05) means reject
    GPD hypothesis; the tail is NOT well-modelled by GPD.
    """
    if not HAS_SCIPY or math.isnan(shape) or math.isnan(scale):
        return float("nan"), float("nan")
    try:
        stat, p = kstest(excess, "genpareto", args=(shape, 0, scale))
        return float(stat), float(p)
    except Exception:
        return float("nan"), float("nan")


def predicted_excess_mean(shape: float, scale: float) -> float:
    """GPD-predicted E[excess] given shape and scale.

    Formula: sigma / (1 - xi) for xi < 1. Infinite for xi >= 1.
    """
    if math.isnan(shape) or math.isnan(scale):
        return float("nan")
    if shape >= 1.0:
        return float("inf")
    return scale / (1.0 - shape)


# --------------------------------------------------------------------
# ULAoI analysis per profile
# --------------------------------------------------------------------

def analyze_profile(aoi_s: np.ndarray, threshold_s: float,
                    profile_name: str = "") -> Dict[str, float]:
    """Full per-profile ULAoI analysis."""
    aoi_s = np.asarray(aoi_s, dtype=np.float64)
    aoi_s = aoi_s[~np.isnan(aoi_s)]
    aoi_s = np.abs(aoi_s)

    result: Dict[str, float] = {
        "profile": profile_name,
        "n_samples": len(aoi_s),
        "aoi_mean_ms": float(np.mean(aoi_s) * 1000) if len(aoi_s) else float("nan"),
        "aoi_p95_ms": float(np.quantile(aoi_s, 0.95) * 1000) if len(aoi_s) else float("nan"),
        "aoi_p99_ms": float(np.quantile(aoi_s, 0.99) * 1000) if len(aoi_s) else float("nan"),
    }

    if len(aoi_s) == 0:
        return result

    excess = aoi_s[aoi_s > threshold_s] - threshold_s
    n_excess = len(excess)

    result.update({
        "threshold_ms": threshold_s * 1000,
        "p_violation": float(n_excess) / len(aoi_s),
        "n_excess": n_excess,
        "excess_mean_ms_empirical": float(np.mean(excess) * 1000) if n_excess else float("nan"),
    })

    # MoM
    shape_mom, scale_mom = fit_gpd_mom(excess)
    result["gpd_shape_mom"] = shape_mom
    result["gpd_scale_mom"] = scale_mom

    # MLE
    shape_mle, scale_mle, mle_warn = fit_gpd_mle(excess)
    result["gpd_shape_mle"] = shape_mle
    result["gpd_scale_mle"] = scale_mle
    result["gpd_mle_warning"] = mle_warn or ""

    # Predicted vs empirical mean excess (MLE)
    pred = predicted_excess_mean(shape_mle, scale_mle)
    result["excess_mean_ms_gpd_predicted"] = pred * 1000 if not math.isinf(pred) else float("inf")

    # KS test against MLE fit
    if not math.isnan(shape_mle):
        ks_stat, ks_p = gpd_ks_test(excess, shape_mle, scale_mle)
        result["ks_stat"] = ks_stat
        result["ks_pvalue"] = ks_p
        result["gpd_fits"] = "yes" if ks_p > 0.05 else "no (reject GPD hypothesis)"
    else:
        result["ks_stat"] = float("nan")
        result["ks_pvalue"] = float("nan")
        result["gpd_fits"] = "insufficient data"

    return result


# --------------------------------------------------------------------
# Mean Residual Life plot (threshold selection diagnostic)
# --------------------------------------------------------------------

def mean_residual_life(aoi_s: np.ndarray,
                       thresholds_s: np.ndarray) -> np.ndarray:
    """For each candidate threshold, compute mean excess above it.

    In peaks-over-threshold theory, the MRL plot is linear in the
    region where GPD is a valid tail model. Choose the lowest
    threshold above which the plot looks linear.
    """
    mrl = np.full_like(thresholds_s, np.nan, dtype=np.float64)
    for i, thr in enumerate(thresholds_s):
        exc = aoi_s[aoi_s > thr] - thr
        if len(exc) >= 20:  # need some samples for stability
            mrl[i] = np.mean(exc)
    return mrl


def plot_mrl(aoi_s: np.ndarray, profile_name: str, out_path: Path) -> None:
    """Save a mean-residual-life plot for this profile."""
    if not HAS_MPL:
        return
    aoi_s = np.abs(np.asarray(aoi_s, dtype=np.float64))
    aoi_s = aoi_s[~np.isnan(aoi_s)]
    if len(aoi_s) < 50:
        return
    thr_max = float(np.quantile(aoi_s, 0.95))
    thresholds = np.linspace(0.0, thr_max, 50)
    mrl = mean_residual_life(aoi_s, thresholds)

    fig, ax = plt.subplots(figsize=(5, 3))
    ax.plot(thresholds * 1000, mrl * 1000, marker=".", lw=1)
    ax.set_xlabel("Threshold τ (ms)")
    ax.set_ylabel("E[AoI − τ | AoI > τ] (ms)")
    ax.set_title(f"Mean residual life · {profile_name}")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


# --------------------------------------------------------------------
# Synthetic validation mode
# --------------------------------------------------------------------

PROFILE_MODELS = {
    # Delay mean, jitter (std), loss_fraction -> AoI model.
    # AoI here is modelled as M/M/1-like: exponential with rate tuned to
    # give roughly the observed profile means from pilot data.
    "ideal":           {"mean_ms": 50,  "heavy_tail_shape": 0.0, "jitter_ms": 20},
    "lan":             {"mean_ms": 55,  "heavy_tail_shape": 0.0, "jitter_ms": 22},
    "wifi_good":       {"mean_ms": 60,  "heavy_tail_shape": 0.1, "jitter_ms": 30},
    "wifi_congested":  {"mean_ms": 120, "heavy_tail_shape": 0.3, "jitter_ms": 80},
    "4g":              {"mean_ms": 100, "heavy_tail_shape": 0.25, "jitter_ms": 50},
    "poor_4g":         {"mean_ms": 180, "heavy_tail_shape": 0.4, "jitter_ms": 120},
    "satellite":       {"mean_ms": 450, "heavy_tail_shape": 0.5, "jitter_ms": 180},
}


def synthesize_profile(name: str, n: int = 3000,
                       seed: int = 42) -> np.ndarray:
    """Generate synthetic AoI samples mimicking real testbed data."""
    rng = np.random.default_rng(seed)
    params = PROFILE_MODELS[name]
    mean = params["mean_ms"] / 1000.0
    shape = params["heavy_tail_shape"]
    jitter = params["jitter_ms"] / 1000.0

    if shape < 0.05:
        # Near-Gaussian regime
        samples = rng.normal(mean, jitter, size=n)
        samples = np.abs(samples)
    else:
        # Mix lognormal bulk with GPD tail
        bulk = rng.lognormal(mean=np.log(mean) - 0.5 * 0.3 ** 2,
                             sigma=0.3, size=n)
        if HAS_SCIPY:
            tail_frac = 0.2
            n_tail = int(n * tail_frac)
            tail = genpareto.rvs(c=shape, loc=mean,
                                 scale=mean * 1.5, size=n_tail,
                                 random_state=rng)
            samples = np.concatenate([bulk[:n - n_tail], tail])
            rng.shuffle(samples)
        else:
            samples = bulk
    return samples


def run_synthetic(threshold_ms: float = 200.0) -> None:
    print(f"\n=== Synthetic ULAoI validation, τ = {threshold_ms} ms ===\n")
    header = ("Profile         | n    | P[Δ>τ] | E_emp | E_GPD | ξ_MLE | "
              "KS-p | GPD-fit")
    print(header)
    print("-" * len(header))
    for profile in PROFILE_MODELS:
        aoi = synthesize_profile(profile, n=3000)
        r = analyze_profile(aoi, threshold_ms / 1000.0, profile)
        print(f"{profile:15s} | {r['n_samples']:4d} | "
              f"{r['p_violation']:5.2%} | "
              f"{r.get('excess_mean_ms_empirical', float('nan')):5.0f} | "
              f"{r.get('excess_mean_ms_gpd_predicted', float('nan')):5.0f} | "
              f"{r.get('gpd_shape_mle', float('nan')):+5.2f} | "
              f"{r.get('ks_pvalue', float('nan')):4.2f} | "
              f"{r.get('gpd_fits', '?'):s}")


# --------------------------------------------------------------------
# Main: real CSV mode
# --------------------------------------------------------------------

def _drop_transient(df: "pd.DataFrame", profile_col: str,
                    transient_s: float) -> "pd.DataFrame":
    """Drop the first `transient_s` seconds of each profile run.

    aoi_logger writes wall_time_s. We compute, for each contiguous block
    of identical network_profile values, the wall-clock start time and
    keep only rows >= start + transient_s.
    """
    if transient_s <= 0:
        return df
    if "wall_time_s" not in df.columns:
        return df
    df = df.sort_values("wall_time_s").reset_index(drop=True)
    # Each profile change starts a new block. Mark blocks.
    block_id = (df[profile_col] != df[profile_col].shift()).cumsum()
    # Compute per-block start time (in wall-clock seconds).
    block_start = df.groupby(block_id)["wall_time_s"].transform("min")
    keep = (df["wall_time_s"] - block_start) >= transient_s
    return df[keep].reset_index(drop=True)


def run_on_csv(csv_path: Path, threshold_ms: float, mrl_plots: bool,
               transient_s: float = 2.0) -> List[Dict[str, float]]:
    if not HAS_PANDAS:
        print("pandas required for CSV mode"); return []
    df = pd.read_csv(csv_path)
    candidate_cols = ["aoi_s", "aoi_abs_s", "aoi_twin_s", "aoi_real_s"]
    aoi_col = None
    for c in candidate_cols:
        if c in df.columns:
            aoi_col = c
            break
    if aoi_col is None:
        print(f"ERROR: could not find AoI column. Tried: {candidate_cols}")
        print(f"Available columns: {list(df.columns)[:20]}")
        return []
    print(f"Using AoI column: {aoi_col}")

    profile_col = None
    for cand in ("network_profile", "profile", "tc_profile", "net_profile"):
        if cand in df.columns:
            profile_col = cand
            break
    if profile_col is None:
        print("WARNING: no profile column found — treating whole CSV as one profile")
        df["profile"] = "combined"
        profile_col = "profile"
    else:
        print(f"Using profile column: {profile_col}")

    if transient_s > 0:
        n_before = len(df)
        df = _drop_transient(df, profile_col, transient_s)
        print(f"Dropped {n_before - len(df)} transient rows "
              f"({transient_s}s after each profile change)")

    threshold_s = threshold_ms / 1000.0

    print(f"\n=== ULAoI analysis of {csv_path.name}, τ = {threshold_ms} ms ===\n")
    header = ("Profile         | n    | P[Δ>τ] | E_emp | E_GPD | ξ_MLE | "
              "KS-p  | GPD-fit")
    print(header)
    print("-" * len(header))

    results = []
    profile_order = ["ideal", "lan", "wifi_good", "wifi_congested",
                     "4g", "poor_4g", "satellite"]
    seen_profiles = set(df[profile_col].unique())
    profiles = [p for p in profile_order if p in seen_profiles]
    profiles += sorted(seen_profiles - set(profiles))

    for profile in profiles:
        sub = df[df[profile_col] == profile]
        aoi = sub[aoi_col].dropna().to_numpy()
        r = analyze_profile(aoi, threshold_s, str(profile))
        r["sweep"] = csv_path.stem
        results.append(r)
        print(f"{str(profile):15s} | {r['n_samples']:4d} | "
              f"{r.get('p_violation', float('nan')):5.2%} | "
              f"{r.get('excess_mean_ms_empirical', float('nan')):5.0f} | "
              f"{r.get('excess_mean_ms_gpd_predicted', float('nan')):5.0f} | "
              f"{r.get('gpd_shape_mle', float('nan')):+5.2f} | "
              f"{r.get('ks_pvalue', float('nan')):.3f} | "
              f"{r.get('gpd_fits', '?'):s}")

        if mrl_plots:
            out_path = csv_path.with_name(
                f"mrl_{profile}_{csv_path.stem}.png")
            plot_mrl(aoi, str(profile), out_path)
            if HAS_MPL:
                print(f"   MRL plot saved: {out_path.name}")

    return results


def run_pooled(csv_paths: List[Path], threshold_ms: float, mrl_plots: bool,
               transient_s: float = 2.0) -> None:
    """Pool AoI samples across all sweeps per profile, then analyse.

    This is the right level for ULAoI claims in the paper: each profile's
    statistical properties are characterized using all available samples,
    not per-sweep.
    """
    if not HAS_PANDAS:
        print("pandas required"); return

    print(f"\n=== POOLED ULAoI across {len(csv_paths)} sweeps ===\n")

    pooled_per_profile: Dict[str, List[float]] = {}
    for path in csv_paths:
        df = pd.read_csv(path)
        candidate_cols = ["aoi_s", "aoi_abs_s"]
        aoi_col = next((c for c in candidate_cols if c in df.columns), None)
        if aoi_col is None:
            print(f"  skipping {path.name}: no AoI column")
            continue
        profile_col = next((c for c in
                            ("network_profile", "profile",
                             "tc_profile", "net_profile")
                            if c in df.columns), None)
        if profile_col is None:
            print(f"  skipping {path.name}: no profile column")
            continue
        if transient_s > 0:
            df = _drop_transient(df, profile_col, transient_s)
        for profile, sub in df.groupby(profile_col):
            pooled_per_profile.setdefault(str(profile), []).extend(
                sub[aoi_col].dropna().tolist())

    threshold_s = threshold_ms / 1000.0
    header = ("Profile         | n     | P[Δ>τ] | E_emp | E_GPD | ξ_MLE | "
              "KS-p  | GPD-fit")
    print(header)
    print("-" * len(header))

    profile_order = ["ideal", "lan", "wifi_good", "wifi_congested",
                     "4g", "poor_4g", "satellite"]
    profiles = [p for p in profile_order if p in pooled_per_profile]
    profiles += sorted(set(pooled_per_profile) - set(profiles))

    for profile in profiles:
        aoi = np.array(pooled_per_profile[profile], dtype=np.float64)
        r = analyze_profile(aoi, threshold_s, profile)
        print(f"{profile:15s} | {r['n_samples']:5d} | "
              f"{r.get('p_violation', float('nan')):5.2%} | "
              f"{r.get('excess_mean_ms_empirical', float('nan')):5.0f} | "
              f"{r.get('excess_mean_ms_gpd_predicted', float('nan')):5.0f} | "
              f"{r.get('gpd_shape_mle', float('nan')):+5.2f} | "
              f"{r.get('ks_pvalue', float('nan')):.3f} | "
              f"{r.get('gpd_fits', '?'):s}")
        if mrl_plots:
            out_path = csv_paths[0].parent / f"mrl_pooled_{profile}.png"
            plot_mrl(aoi, profile, out_path)

    print("\nInterpretation notes:")
    print("  • ξ_MLE > 0     : heavy-tailed (exceedances common)")
    print("  • ξ_MLE ≈ 0     : exponential tail")
    print("  • KS-p > 0.05   : GPD consistent with observed tail")
    print("  • KS-p < 0.05   : GPD NOT a good model — report caveat")
    print("  • E_emp vs E_GPD: agreement between empirical mean-excess")
    print("                     and GPD-predicted mean-excess")


def main():
    p = argparse.ArgumentParser(description="Rigorous ULAoI analysis")
    p.add_argument("csvs", nargs="*", help="enriched CSVs (skip for --synthetic)")
    p.add_argument("--threshold-ms", type=float, default=200.0,
                   help="ULAoI threshold τ in ms (default 200)")
    p.add_argument("--synthetic", action="store_true",
                   help="run synthetic validation instead of real CSV")
    p.add_argument("--mrl-plots", action="store_true",
                   help="save mean-residual-life plots per profile")
    p.add_argument("--pool", action="store_true",
                   help="pool samples across all input CSVs per profile "
                        "(use for paper-grade per-profile statistics)")
    p.add_argument("--transient-s", type=float, default=2.0,
                   help="seconds to drop after each profile change "
                        "(default 2.0; matches aggregate_runs convention)")
    args = p.parse_args()

    if args.synthetic or not args.csvs:
        run_synthetic(args.threshold_ms)
        return

    paths = [Path(c) for c in args.csvs]

    if args.pool:
        run_pooled(paths, args.threshold_ms, args.mrl_plots,
                   transient_s=args.transient_s)
    else:
        for path in paths:
            print()
            run_on_csv(path, args.threshold_ms, args.mrl_plots,
                       transient_s=args.transient_s)


if __name__ == "__main__":
    main()
