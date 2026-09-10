# kaoi-testbed

Hardware testbed and analysis code for **K-AoI (Kinematic Age of Information)**, a
velocity-weighted Age of Information metric for digital-twin robot teleoperation.

K-AoI is defined as the product of Age of Information and the end-effector Cartesian
speed, giving a staleness measure in metres rather than seconds: the distance the
end-effector has travelled since the last state update. This repository contains the
measurement pipeline behind the paper's results: the ROS 2 package that runs the twin,
the robot bridge, the kernel-level network impairment, and the logger, plus the Unity
digital-twin scripts, the raw and enriched experiment logs from all 11 trials, and the
analysis scripts that regenerate the paper's tables and figures from that data.

The physical setup is a UFactory Lite 6 (6-DOF) manipulator commanded from a Unity
2022.3 digital twin over a ROS 2 bridge, with `tc netem` applied at the kernel level to
emulate seven network conditions ranging from an ideal LAN to a GEO satellite link.

---

## Repository layout

```
kaoi-testbed/
├── ros2_ws/src/dt_teleop_testbed/     ROS 2 (ament_python) package (the testbed)
│   ├── dt_teleop_testbed/             Nodes
│   ├── scripts/                       Analysis, figure, and table-verification scripts
│   ├── launch/testbed.launch.py       Single-command bring-up
│   ├── config/network_profiles.yaml   tc netem profile definitions
│   ├── unity_scripts/                 Unity C# scripts (installed to the ROS share dir)
│   └── docs/unity_brick_setup.md      ArUco brick tracking setup notes
├── unity/Assets/Scripts/              Unity C# scripts (drop into your Unity project)
├── paper/figures/testbed.tex          TikZ source for the testbed architecture figure
│                                      (needs the six icon_*.png files alongside it)
├── data/
│   ├── raw/                           11 experiment runs: logger CSV + enriched CSV
│   └── aggregated/                    Cross-run summary JSON and paper figures
├── LICENSE                            MIT
└── README.md
```

The five slider-mode Unity scripts are byte-identical in `unity/Assets/Scripts/` and
`ros2_ws/src/dt_teleop_testbed/unity_scripts/`. The ROS copy exists because `setup.py`
installs it into the package share directory; the `unity/` copy is the one to drop into
a Unity project.

---

## Requirements

**Host (Linux, ROS 2 side)**

| Component | Version used |
|---|---|
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| Python | 3.12 |
| `iproute2` (`tc`) | for netem impairment |

Python packages for the analysis scripts:

```bash
pip3 install numpy pandas matplotlib scipy pyyaml
```

**Robot side (only for real-hardware runs)**

- UFactory Lite 6 on the same subnet as the host
- [`xarm_ros2`](https://github.com/xArm-Developer/xarm_ros2) (`xarm_api`, `xarm_msgs`)
  built in the same workspace

**Unity side (only when Unity drives the twin)**

- Unity 2022.3
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) (Unity)
  and [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) (ROS side)
- Lite 6 URDF imported via URDF-Importer

Nothing in this repo requires the robot or Unity: the testbed runs end-to-end in mock
mode on a single laptop.

---

## Installation

```bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/luziyaotian/kaoi-testbed.git
cp -r kaoi-testbed/ros2_ws/src/dt_teleop_testbed ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select dt_teleop_testbed
source install/setup.bash
```

`tc` needs privileges to change the queueing discipline. Either grant the capability
once (preferred, avoids `sudo` inside the launch file):

```bash
sudo setcap cap_net_admin+ep $(readlink -f $(which tc))
```

…or pass `use_sudo_tc:=true` at launch and run from a shell with a warm `sudo` timestamp.

Check the environment before a real run:

```bash
python3 ros2_ws/src/dt_teleop_testbed/scripts/preflight.py --robot-ip 192.168.1.167 --iface <your_iface> --full
```

---

## Running the testbed

Everything comes up from one launch file. Node topology:

```
twin source ──/twin/joint_states──▶ bridge ──▶ Lite 6 ──/lite6_real/joint_states──┐
  (Unity, headless_twin, or                                                        │
   scripted_operator)                                                              ▼
                                                                              aoi_logger ──▶ CSV
network_conditioner ──/network_profile──▶ (tags each logged row)
```

`tc netem` is applied to the egress interface, so impairment affects the twin→robot
command path.

### Mock mode (no hardware needed)

```bash
ros2 launch dt_teleop_testbed testbed.launch.py \
    use_mock:=true \
    interface:=<your_iface>
```

`mock_lite6` simulates first-order joint dynamics so the "robot" cannot instantaneously
reach commanded poses. Add `skip_netem:=true` to run without touching the network stack
at all.

### Real Lite 6

```bash
ros2 launch dt_teleop_testbed testbed.launch.py \
    use_mock:=false \
    robot_ip:=192.168.1.167 \
    interface:=<your_iface>
```

This includes the `xarm_api` Lite 6 driver launch and starts `twin_to_lite6_bridge`,
which rate-limits twin poses to `forward_rate_hz` before sending them to the controller.

### Unity as the twin

```bash
ros2 launch dt_teleop_testbed testbed.launch.py \
    use_unity:=true \
    unity_ip:=<ip Unity connects from> \
    unity_port:=10000 \
    robot_ip:=192.168.1.167 \
    interface:=<your_iface>
```

This starts ROS-TCP-Endpoint instead of `headless_twin`; Unity then publishes
`/twin/joint_states` itself.

### Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `use_mock` | `false` | Use `mock_lite6` instead of the real driver + bridge |
| `interface` | - | Ethernet interface for `tc netem`; required unless `skip_netem:=true` |
| `robot_ip` | `192.168.1.167` | Lite 6 controller IP. **Change to your robot's** |
| `twin_mode` | `sine` | `headless_twin` trajectory: `sine` \| `waypoints` \| `replay` |
| `skip_netem` | `false` | Don't start `network_conditioner` |
| `skip_logger` | `false` | Don't start `aoi_logger` |
| `commander_kind` | `planner` | Bridge command strategy: `planner` \| `service` |
| `forward_rate_hz` | `10.0` | Rate at which the bridge forwards twin poses to the robot |
| `csv_path` | `""` | Logger output path; auto-generated if empty |
| `use_sudo_tc` | `false` | Set `true` if `tc` was not given `cap_net_admin` |
| `use_unity` | `false` | Start ROS-TCP-Endpoint and let Unity be the twin |
| `unity_ip` / `unity_port` | `127.0.0.1` / `10000` | Endpoint bind address |
| `use_gripper` | `false` | Start `gripper_bridge` (real robot only) |
| `use_scripted` | `false` | Use `scripted_operator` as the twin source (zero operator variance) |

### Nodes

| Executable | Role |
|---|---|
| `headless_twin` | Publishes `/twin/joint_states` from a scripted trajectory; the twin "data contract" |
| `scripted_operator` | Reproducible multi-joint trajectory at 50 Hz, seeded from the real robot pose |
| `mock_lite6` | Stand-in robot with first-order joint dynamics |
| `twin_to_lite6_bridge` | Rate-limits and forwards twin poses to the real controller |
| `network_conditioner` | Applies `tc netem` profiles; listens on `/network_profile_request`, echoes on `/network_profile` |
| `aoi_logger` | Samples twin and real state at 20 Hz, computes AoI and joint-space error, writes CSV |
| `experiment_runner` | Steps through the profile sequence for a full sweep |
| `gripper_bridge` | Maps `std_msgs/Bool` on `/gripper/command` to the Lite 6 gripper services |
| `aruco_detector` | RealSense RGB-D ArUco marker pose detection for object tracking |

---

## Network profiles

Defined in `config/network_profiles.yaml` and applied to the egress interface with
`tc netem`. Delays are **one-way**.

| Profile | Delay (ms) | Jitter (ms) | Loss (%) |
|---|---|---|---|
| `ideal` | - | - | - |
| `lan` | 1 | 0.2 | - |
| `wifi_good` | 10 | 2 | - |
| `wifi_congested` | 50 | 10 | 1 |
| `4g` | 60 | 15 | 0.5 |
| `poor_4g` | 200 | 50 | 2 |
| `satellite` | 600 | 100 | 0.5 |

Switch profile manually at any time:

```bash
ros2 topic pub --once /network_profile_request std_msgs/String "{data: 'poor_4g'}"
```

The conditioner applies `ideal` on startup and clears the qdisc on shutdown.

---

## Running an experiment sweep

With the testbed already up (terminal 1), run the sweep in terminal 2:

```bash
ros2 run dt_teleop_testbed experiment_runner
```

The default sequence visits eight segments,
`ideal → lan → wifi_good → wifi_congested → 4g → poor_4g → satellite → ideal`, at
`dwell_s` = 30 s each, so a full run is 4 minutes. The trailing `ideal` bookends the
sweep so the baseline is measured under the same operator behaviour at both ends and the
network is left clean on exit. `Ctrl-C` re-applies `ideal` before quitting.

Override the sequence or dwell time:

```bash
ros2 run dt_teleop_testbed experiment_runner --ros-args \
    -p dwell_s:=60.0 \
    -p profiles:="['ideal','satellite','ideal']"
```

One run produces one CSV at `~/testbed_logs/aoi_<YYYYmmdd>_<HHMMSS>.csv`, with every row
tagged by the profile in force at that moment.

---

## Analysis pipeline

All scripts live in `ros2_ws/src/dt_teleop_testbed/scripts/` and run standalone; no ROS
environment needed.

**1. Enrich a raw log.** Adds forward kinematics, Cartesian divergence, end-effector
speed, and K-AoI to each row, and writes a per-profile summary JSON:

```bash
python3 ros2_ws/src/dt_teleop_testbed/scripts/compute_metrics.py data/raw/E3_20260425_152014.csv \
    --aoi-threshold-ms 100 \
    --profile-transient-s 2.0
```

`--profile-transient-s 2.0` discards the first 2 seconds after each profile change, while
`tc` settles and the queue drains. **Use the same value everywhere**: sample counts and
correlation coefficients will not match the paper otherwise.

**2. Aggregate across runs.** Per-profile mean ± 95% CI over all 11 runs:

```bash
python3 ros2_ws/src/dt_teleop_testbed/scripts/aggregate_runs.py data/raw/*_enriched.csv \
    --out-dir data/aggregated \
    --threshold-ms 200
```

**3. Figures and tail analysis.**

```bash
# Per-run figures: timeline, CCDF, K-AoI vs Cartesian error, per-profile bars
python3 ros2_ws/src/dt_teleop_testbed/scripts/plot_metrics.py data/raw/E3_20260425_152014_enriched.csv \
    --summary <summary.json> --out-dir data/aggregated

# Quick-look plots straight from a raw logger CSV
python3 ros2_ws/src/dt_teleop_testbed/scripts/plot_experiment.py data/raw/E3_20260425_152014.csv

# AoI vs K-AoI per-sample scatter, with Spearman rank correlation
python3 ros2_ws/src/dt_teleop_testbed/scripts/kaoi_vs_aoi_scatter.py data/raw/*_enriched.csv

# ULAoI: GPD tail fit by MLE, KS goodness-of-fit, mean-residual-life plots
python3 ros2_ws/src/dt_teleop_testbed/scripts/ulaoi_analysis.py data/raw/*_enriched.csv \
    --threshold-ms 200 --transient-s 2.0 --mrl-plots

# Validate the ULAoI pipeline on synthetic data with known parameters
python3 ros2_ws/src/dt_teleop_testbed/scripts/ulaoi_analysis.py --synthetic
```

Note the differing threshold defaults: `compute_metrics.py` and `plot_metrics.py` default
to 100 ms, `aggregate_runs.py` and `ulaoi_analysis.py` to 200 ms. Pass `--threshold-ms`
explicitly to avoid mixing them.

---

## Reproducing the paper's tables and figures

All commands run from the repository root against the committed data and were verified to
reproduce the published values. The 2-second transient filter is applied throughout.

**Table 3 (headline metrics: AoI P99, P_τ, E_τ, K-AoI P95).** Recomputes every cell two
ways, per-trial-then-averaged (what the table reports) and pooled, and flags any
disagreement with the published values:

```bash
python3 ros2_ws/src/dt_teleop_testbed/scripts/verify_table3.py \
    data/raw/*_enriched.csv --threshold-ms 200 --transient-s 2.0
```

**Table 4 (per-profile Spearman ρ between AoI and K-AoI).** Also reports p-values,
Pearson r as a cross-check, and per-trial ρ mean ± std for robustness:

```bash
python3 ros2_ws/src/dt_teleop_testbed/scripts/verify_table4.py \
    data/raw/*_enriched.csv --transient-s 2.0
```

**Figure 2 (per-profile bars: AoI P99, ULAoI E_τ, K-AoI P95, with 95% CI).** From the
committed summary, or recomputed from scratch with `--csvs`:

```bash
python3 ros2_ws/src/dt_teleop_testbed/scripts/make_figure2_aggregate_bars.py \
    --summary data/aggregated/aggregate_summary.json --out-dir data/aggregated
```

**Figure 3 (AoI vs K-AoI scatter, ideal and satellite, coloured by twin velocity).**
Reproduces the canonical pooled samples and correlations
(ideal: n = 33,852, ρ = +0.295; satellite: n = 6,156, ρ = +0.219):

```bash
python3 ros2_ws/src/dt_teleop_testbed/scripts/make_figure3_scatter.py \
    data/raw/*_enriched.csv --out-dir data/aggregated --transient-s 2.0
```

Spearman ρ is computed on the full pooled sample before any downsampling; downsampling
(`--max-points`, default 8000 per panel, fixed seed) affects only which points are drawn.

---

## Data

`data/raw/` holds 11 complete sweeps, each as a pair: the logger output
(`E3_<timestamp>.csv`) and the enriched version produced by `compute_metrics.py`
(`E3_<timestamp>_enriched.csv`). `data/aggregated/` holds the cross-run summary JSON and
the generated figures. See [`data/README.md`](data/README.md) for the full column
schema and metric definitions.

---

## Unity setup

Copy `unity/Assets/Scripts/*.cs` into your Unity project's `Assets/Scripts/`, import the
Lite 6 URDF with URDF-Importer, and attach the scripts to the imported robot root
(`UF_ROBOT`).

| Script | Role |
|---|---|
| `TwinJointStatePublisher.cs` | Publishes the twin's 6 joint angles as `sensor_msgs/JointState` on `/twin/joint_states`; the Unity replacement for `headless_twin` |
| `JointSliderController.cs` | On-screen 6-slider jog panel; seeds from `/ufactory/joint_states` on Play so the twin doesn't jerk to zero |
| `RealRobotStateSubscriber.cs` | Drives a second, transparent ArticulationBody chain from `/lite6_real/joint_states` so the operator can see the real robot lagging behind. Diagnostic only; the ROS-side logger is the authoritative instrument |
| `GripperController.cs` | Open/close toggle publishing `std_msgs/Bool` on `/gripper/command` (keyboard: `G`) |
| `BrickPoseSubscriber.cs` | Moves a scene object to match `/aruco/marker_0_pose`, handling the OpenCV→Unity frame conversion |
| `Editor/ArticulationDriveSetup.cs` | Editor utility (right-click → *Robot Utilities/Configure Articulation Drives*) that writes drive stiffness and damping into the scene file. URDF-Importer leaves these at zero, so joints sag under gravity without it |

`docs/unity_brick_setup.md` covers the ArUco brick-tracking scene setup.

---

## Notes and known limitations

- **The bridge is a rate limiter, not a transport.** `twin_to_lite6_bridge` forwards at
  `forward_rate_hz`, which sets a floor on twin-to-robot lag independent of network
  conditions. This dominates measured Cartesian divergence at low impairment levels.
- **`aoi_s` equals `aoi_real_s`**, the freshness of real-robot joint state as seen at
  the twin. `aoi_twin_s` is dominated by Unity-side frame stalls and is not affected by
  `tc netem`; do not use it as the network-staleness signal.
- **`tc netem` applies to egress only.** Round-trip delay is roughly twice the configured
  `delay_ms` when both directions traverse the impaired interface.
- **The 2-second transient filter is not optional.** Every reported statistic applies it.
- The `unity_scripts/` and `unity/Assets/Scripts/` copies are duplicated (see
  [Repository layout](#repository-layout)).

---

## Citation

The K-AoI paper has been accepted at the 2026 IEEE International Conference on Digital
Twin (IEEE SWC 2026, Rende, Italy) and will appear in the proceedings. Until it is
published in IEEE Xplore, please cite it as:

```bibtex
@inproceedings{tian2026kaoi,
  author    = {Tian, Lu and Deng, Zexin and Yuan, Zhenhui},
  title     = {{K-AoI}: A Spatio-Temporal Synchronization Metric for
               Digital Twin-based Teleoperation in Dynamic Network
               Environments},
  booktitle = {2026 IEEE International Conference on Digital Twin},
  year      = {2026},
  note      = {To appear}
}
```
```

## License

MIT. See [LICENSE](LICENSE).
