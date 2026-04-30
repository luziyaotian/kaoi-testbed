#!/usr/bin/env python3
"""
kaoi_vs_aoi_scatter.py — per-sample comparison of AoI vs K-AoI.

For the paper's claim "K-AoI captures degradation invisible to AoI":
plot AoI(t) on x-axis vs K-AoI(t) on y-axis, one point per timestamp.

If AoI fully determined K-AoI, the points would lie on a single curve.
The vertical spread at any given AoI tells us about velocity variation:
that spread is the additional information K-AoI carries beyond AoI.

Outputs:
  - kaoi_vs_aoi_<profile>.png : scatter coloured by twin velocity
  - prints Spearman rank correlation (high = metrics agree;
    moderate = metrics disagree, K-AoI carries extra info)

Usage:
    python3 kaoi_vs_aoi_scatter.py enriched.csv [enriched2.csv ...]
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.stats import spearmanr


PROFILE_ORDER = ["ideal", "lan", "wifi_good", "wifi_congested",
                 "4g", "poor_4g", "satellite"]


def analyse_one_profile(df: pd.DataFrame, profile: str,
                        out_dir: Path) -> dict:
    sub = df[df["network_profile"] == profile].copy()
    if len(sub) < 100:
        return {"profile": profile, "n": len(sub), "skipped": True}

    aoi_ms = sub["aoi_s"].abs().to_numpy() * 1000
    kaoi_mm = sub["k_aoi_eef_max_m"].to_numpy() * 1000  # m -> mm
    velocity = sub["twin_velocity_mps"].to_numpy()

    # Drop any NaNs
    mask = np.isfinite(aoi_ms) & np.isfinite(kaoi_mm) & np.isfinite(velocity)
    aoi_ms = aoi_ms[mask]
    kaoi_mm = kaoi_mm[mask]
    velocity = velocity[mask]

    if len(aoi_ms) < 50:
        return {"profile": profile, "n": len(aoi_ms), "skipped": True}

    # Spearman rank correlation: high = K-AoI is just AoI rescaled,
    # moderate = K-AoI has independent information.
    rho, p = spearmanr(aoi_ms, kaoi_mm)

    # Plot: scatter coloured by velocity
    fig, ax = plt.subplots(figsize=(5.5, 4))
    sc = ax.scatter(aoi_ms, kaoi_mm, c=velocity * 1000, cmap="viridis",
                    s=4, alpha=0.5)
    cb = fig.colorbar(sc, ax=ax)
    cb.set_label("Twin velocity (mm/s)")
    ax.set_xlabel("AoI (ms)")
    ax.set_ylabel("K-AoI$_\\mathrm{eef}$ (mm)")
    ax.set_title(f"{profile}  ·  ρ = {rho:.3f}  ·  n = {len(aoi_ms)}")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out_path = out_dir / f"kaoi_vs_aoi_{profile}.png"
    fig.savefig(out_path, dpi=120)
    plt.close(fig)

    return {
        "profile": profile,
        "n": int(len(aoi_ms)),
        "spearman_rho": float(rho),
        "spearman_p": float(p),
        "aoi_p50_ms": float(np.median(aoi_ms)),
        "aoi_p95_ms": float(np.quantile(aoi_ms, 0.95)),
        "kaoi_p50_mm": float(np.median(kaoi_mm)),
        "kaoi_p95_mm": float(np.quantile(kaoi_mm, 0.95)),
        "velocity_p50_mmps": float(np.median(velocity) * 1000),
        "velocity_p95_mmps": float(np.quantile(velocity, 0.95) * 1000),
        "plot_path": str(out_path),
    }


def main():
    if len(sys.argv) < 2:
        print("Usage: kaoi_vs_aoi_scatter.py enriched.csv [enriched2.csv ...]")
        sys.exit(1)

    csvs = [Path(p) for p in sys.argv[1:]]
    out_dir = csvs[0].parent / "kaoi_vs_aoi_analysis"
    out_dir.mkdir(exist_ok=True)

    # Pool all CSVs into one dataframe (per-profile pooled across sweeps)
    print(f"Loading {len(csvs)} CSVs...")
    dfs = []
    for c in csvs:
        try:
            d = pd.read_csv(c)
            d["_source"] = c.name
            dfs.append(d)
        except Exception as exc:
            print(f"  skip {c.name}: {exc}")
    if not dfs:
        print("No CSVs loaded"); sys.exit(1)
    df = pd.concat(dfs, ignore_index=True)
    print(f"Total rows: {len(df)}")

    profiles_seen = list(df["network_profile"].dropna().unique())
    profiles = [p for p in PROFILE_ORDER if p in profiles_seen]
    profiles += sorted(set(profiles_seen) - set(profiles))

    print()
    print("Per-profile AoI vs K-AoI comparison:")
    print()
    header = ("Profile         |    n    | Spearman ρ | AoI P95 | K-AoI P95 | "
              "vel P95")
    print(header)
    print("-" * len(header))
    print(" " * len(header.split('|')[0]) + "|" + " " * 9 +
          "| (high ρ ≈     |   (ms)  |   (mm)    |  (mm/s)")
    print(" " * len(header.split('|')[0]) + "|" + " " * 9 +
          "| metrics agree)|         |           |")
    print("-" * len(header))

    results = []
    for profile in profiles:
        r = analyse_one_profile(df, profile, out_dir)
        if r.get("skipped"):
            print(f"{profile:15s} | {r['n']:6d}  | (insufficient data)")
            continue
        results.append(r)
        print(f"{profile:15s} | {r['n']:6d}  | "
              f"{r['spearman_rho']:+.3f}     | "
              f"{r['aoi_p95_ms']:6.1f}  | "
              f"{r['kaoi_p95_mm']:7.1f}   | "
              f"{r['velocity_p95_mmps']:6.1f}")

    print()
    print("Interpretation:")
    print("  ρ ≈ 1.00  : K-AoI = AoI × constant velocity (no extra info)")
    print("  ρ ≈ 0.95  : nearly redundant, but velocity adds a little")
    print("  ρ < 0.85  : K-AoI carries substantial info beyond AoI")
    print("  ρ < 0.50  : metrics measure largely different things")
    print()
    print(f"Plots written to: {out_dir}/")
    print()
    print("Look at the scatter plots: vertical spread at a given AoI")
    print("indicates how much velocity variation there is at that AoI.")
    print("Wide spread = K-AoI shows what AoI cannot.")


if __name__ == "__main__":
    main()
