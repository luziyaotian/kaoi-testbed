# Data

Experiment logs from the K-AoI testbed. Eleven complete network sweeps, each a single
continuous run of the profile sequence
`ideal → lan → wifi_good → wifi_congested → 4g → poor_4g → satellite → ideal`
at 30 s per segment (4 minutes per run), logged at 20 Hz.

```
data/
├── raw/
│   ├── E3_<YYYYmmdd>_<HHMMSS>.csv            raw aoi_logger output
│   └── E3_<YYYYmmdd>_<HHMMSS>_enriched.csv   after compute_metrics.py
└── aggregated/
    ├── aggregate_summary.json                per-profile stats across all 11 runs
    ├── figure2_aggregate_bars.{pdf,png}
    ├── figure3_aoi_vs_kaoi_scatter.{pdf,png}
    └── E3_20260424_160148_enriched_*.png     per-run figures (timeline, CCDF, …)
```

The enriched files are fully regenerable:

```bash
python3 ../ros2_ws/src/dt_teleop_testbed/scripts/compute_metrics.py raw/E3_<ts>.csv \
    --profile-transient-s 2.0
```

---

## Raw CSV schema

Written by `aoi_logger_node` at 20 Hz. One row per log tick; each row carries the most
recent twin and real-robot state received at that moment.

| Column | Unit | Meaning |
|---|---|---|
| `wall_time_s` | s | High-precision wall clock at the log tick |
| `ros_time_s` | s | ROS time at the log tick |
| `network_profile` | - | `tc netem` profile in force at this tick |
| `twin_stamp_s` | s | `header.stamp` of the most recent twin message |
| `real_stamp_s` | s | `header.stamp` of the most recent real-robot message |
| `aoi_s` | s | **Primary AoI metric.** Alias of `aoi_real_s`, retained for backward compatibility |
| `aoi_twin_s` | s | Time since the logger last received a twin message |
| `aoi_real_s` | s | Time since the logger last received a real-robot message |
| `msg_dt_s` | s | `real_stamp_s − twin_stamp_s` |
| `twin_joint1..6` | rad | Latest twin joint positions |
| `real_joint1..6` | rad | Latest real-robot joint positions |
| `err_joint1..6` | rad | `twin − real`, per joint |
| `err_l2_rad` | rad | L2 norm of the joint error vector |
| `err_max_rad` | rad | `max(|err_jointi|)` |
| `n_twin` | - | Running count of twin messages received |
| `n_real` | - | Running count of real-robot messages received |

### Which AoI column to use

`aoi_real_s` (and therefore `aoi_s`) is the network-sensitive one. It is computed from
the logger's **receive wall-clock time**, not from `header.stamp`, because the xArm
driver stamps messages *after* reading TCP bytes from the robot, i.e. after the kernel
has already released the netem-delayed packet. A `wall − stamp` calculation would
therefore be near-zero regardless of impairment. `wall − rx_wall` is what actually
responds to `tc netem`.

`aoi_twin_s` measures Unity→ROS latency over loopback, which netem does not impair. It is
dominated by Unity-side frame stalls and should **not** be used as the network-staleness
signal.

---

## Enriched CSV schema

`compute_metrics.py` appends the following to every raw column above.

| Column | Unit | Meaning |
|---|---|---|
| `t_rel` | s | `wall_time_s` relative to the first row |
| `twin_x`, `twin_y`, `twin_z` | m | Twin end-effector position from forward kinematics |
| `real_x`, `real_y`, `real_z` | m | Real end-effector position from forward kinematics |
| `cart_err_m` | m | ‖twin_xyz − real_xyz‖, the true Cartesian divergence |
| `twin_velocity_mps` | m/s | Finite-difference speed of the twin end-effector |
| `real_velocity_mps` | m/s | Finite-difference speed of the real end-effector |
| `aoi_abs_s` | s | AoI used for the K-AoI product |
| `k_aoi_eef_twin_m` | m | `twin_velocity_mps × aoi_abs_s` |
| `k_aoi_eef_real_m` | m | `real_velocity_mps × aoi_abs_s` |
| `k_aoi_eef_max_m` | m | `max(k_aoi_eef_twin_m, k_aoi_eef_real_m)`, the reported K-AoI |

Forward kinematics come from `dt_teleop_testbed/lite6_kinematics.py`, whose chain is
extracted directly from the UFactory Lite 6 URDF.

K-AoI is a product, so the two factors are not recoverable from it: a slow-moving arm on
a bad link and a fast-moving arm on a good link can yield the same value. It is a
staleness-in-metres measure, not a joint encoding of latency and speed.

---

## Transient filtering

The first **2 seconds** after every profile change are discarded (`--profile-transient-s
2.0`) while `tc` reconfigures the qdisc and the queue drains. Every statistic in the
paper applies this filter. Re-running any analysis with a different value will change
both sample counts and correlation coefficients; if your numbers do not match the
paper's, check this first.
