#!/usr/bin/env python3
"""
preflight.py — pre-experiment green-light checker.

Runs through every requirement for a successful real-robot experiment, in
order. Prints [OK] / [WARN] / [FAIL] for each, with actionable next steps
on failure.

Usage:
    python3 preflight.py                       # basic checks
    python3 preflight.py --robot-ip 192.168.1.167   # include robot ping
    python3 preflight.py --iface enx00e04c...       # include tc permission check
    python3 preflight.py --robot-ip 192.168.1.167 --iface enx... --full
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Callable, List, Tuple


GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"


class Check:
    def __init__(self, name: str, func: Callable[[], Tuple[str, str]]):
        self.name = name
        self.func = func

    def run(self) -> bool:
        try:
            status, message = self.func()
        except Exception as e:
            status, message = "FAIL", f"exception: {e}"
        colour = {"OK": GREEN, "WARN": YELLOW, "FAIL": RED}.get(status, "")
        print(f"  [{colour}{status:4s}{RESET}] {self.name}")
        if message:
            for line in message.splitlines():
                print(f"         {line}")
        return status != "FAIL"


def run(cmd: List[str], timeout: float = 5.0) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


# ------------------------------------------------------------------ #
# Individual checks
# ------------------------------------------------------------------ #

def check_ros_env() -> Tuple[str, str]:
    distro = os.environ.get("ROS_DISTRO")
    if not distro:
        return "FAIL", "ROS_DISTRO not set.\nFix: source /opt/ros/<distro>/setup.bash"
    if distro != "jazzy":
        return "WARN", f"ROS_DISTRO={distro}; testbed developed on jazzy"
    return "OK", f"ROS_DISTRO={distro}"


def check_ros2_cli() -> Tuple[str, str]:
    if not shutil.which("ros2"):
        return "FAIL", "ros2 CLI not found on PATH"
    return "OK", shutil.which("ros2")


def check_workspace_sourced() -> Tuple[str, str]:
    ampp = os.environ.get("AMENT_PREFIX_PATH", "")
    if "ros2_ws/install" not in ampp:
        return ("FAIL",
                "Workspace not sourced.\n"
                "Fix: source ~/ros2_ws/install/setup.bash")
    return "OK", "ros2_ws in AMENT_PREFIX_PATH"


def check_package(pkg: str) -> Callable[[], Tuple[str, str]]:
    def _check():
        r = run(["ros2", "pkg", "prefix", pkg])
        if r.returncode != 0:
            return ("FAIL",
                    f"Package '{pkg}' not found.\n"
                    f"Fix: colcon build --packages-select {pkg} && "
                    f"source install/setup.bash")
        return "OK", r.stdout.strip()
    return _check


def check_executables(pkg: str, expected: List[str]):
    def _check():
        r = run(["ros2", "pkg", "executables", pkg])
        if r.returncode != 0:
            return "FAIL", f"pkg '{pkg}' not found"
        names = [line.split()[-1] for line in r.stdout.splitlines() if line.strip()]
        missing = [e for e in expected if e not in names]
        if missing:
            return ("FAIL",
                    f"Missing executables: {missing}.\n"
                    f"Fix: rebuild the {pkg} package")
        return "OK", f"{len(expected)} executables registered"
    return _check


def check_tc() -> Tuple[str, str]:
    tc = shutil.which("tc")
    if not tc:
        return "FAIL", "tc not found.\nFix: sudo apt install iproute2"

    # Can we run tc without sudo?
    r = run([tc, "qdisc", "show"])
    if r.returncode == 0:
        return "OK", f"tc usable without sudo ({tc})"

    # Capability check
    caps_r = run(["getcap", tc])
    if "cap_net_admin" in caps_r.stdout:
        return ("WARN",
                "tc has cap_net_admin set but failed to run — "
                "try running preflight as your normal user")

    # Maybe passwordless sudo works?
    r2 = run(["sudo", "-n", tc, "-help"], timeout=3.0)
    if r2.returncode == 0:
        return ("OK",
                f"tc available via passwordless sudo (use_sudo_tc:=true)")

    return ("FAIL",
            "tc requires elevated privileges and neither setcap nor "
            "passwordless sudo is configured.\n"
            "Fix: sudo setcap cap_net_admin+ep $(which tc)")


def check_iface(iface: str):
    def _check():
        if not iface:
            return "WARN", "interface not specified (--iface); tc check skipped"
        sysfs = Path(f"/sys/class/net/{iface}")
        if not sysfs.exists():
            available = sorted(os.listdir("/sys/class/net"))
            return ("FAIL",
                    f"Interface '{iface}' does not exist.\n"
                    f"Available: {available}")
        # Check it's up
        r = run(["ip", "-br", "link", "show", iface])
        if "UP" not in r.stdout.upper():
            return ("WARN",
                    f"Interface '{iface}' exists but is not UP.\n"
                    f"Fix: sudo ip link set {iface} up")
        # Is it in 192.168.1.x range?
        r2 = run(["ip", "-br", "addr", "show", iface])
        if "192.168.1." not in r2.stdout:
            return ("WARN",
                    f"Interface '{iface}' is up but has no 192.168.1.x "
                    f"address. Fix: sudo ip addr add 192.168.1.10/24 "
                    f"dev {iface}")
        return "OK", r2.stdout.strip()
    return _check


def check_ping(robot_ip: str):
    def _check():
        if not robot_ip:
            return "WARN", "robot IP not specified (--robot-ip); ping skipped"
        r = run(["ping", "-c", "2", "-W", "1", robot_ip], timeout=5.0)
        if r.returncode != 0:
            return ("FAIL",
                    f"Cannot ping {robot_ip}.\n"
                    f"Fix: verify ethernet cable, check `ip -br addr` "
                    f"for interface in same subnet")
        return "OK", f"{robot_ip} reachable"
    return _check


def check_robot_studio(robot_ip: str):
    def _check():
        if not robot_ip:
            return "WARN", "skipped (no --robot-ip)"
        try:
            import urllib.request
            req = urllib.request.Request(f"http://{robot_ip}:18333/")
            with urllib.request.urlopen(req, timeout=3) as resp:
                if resp.status == 200:
                    return "OK", "UFACTORY Studio responding on port 18333"
        except Exception as e:
            return ("WARN",
                    f"UFACTORY Studio did not respond on {robot_ip}:18333. "
                    f"Robot may still be booting; wait ~30s and retry.")
        return "WARN", "unexpected response from studio"
    return _check


def check_ros_daemon() -> Tuple[str, str]:
    r = run(["ros2", "daemon", "status"])
    if "online" in r.stdout:
        return "OK", "ros2 daemon online"
    return "WARN", "ros2 daemon not running (will auto-start on first use)"


def check_output_dir() -> Tuple[str, str]:
    log_dir = Path.home() / "testbed_logs"
    if not log_dir.exists():
        log_dir.mkdir(parents=True)
        return "OK", f"created {log_dir}"
    if not os.access(log_dir, os.W_OK):
        return "FAIL", f"{log_dir} not writable"
    return "OK", f"{log_dir} writable"


# ------------------------------------------------------------------ #
# Main
# ------------------------------------------------------------------ #

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="",
                        help="Lite 6 IP for ping + studio check")
    parser.add_argument("--iface", default="",
                        help="Ethernet interface for tc netem")
    parser.add_argument("--full", action="store_true",
                        help="Run all checks (also without ip/iface)")
    args = parser.parse_args()

    print(f"{BOLD}Pre-flight checks for dt_teleop_testbed{RESET}\n")

    checks = [
        Check("ROS 2 environment",          check_ros_env),
        Check("ros2 CLI available",         check_ros2_cli),
        Check("ros2_ws sourced",            check_workspace_sourced),
        Check("ros2 daemon",                check_ros_daemon),
    ]

    print(f"{BOLD}-- Environment --{RESET}")
    results = [c.run() for c in checks]

    print(f"\n{BOLD}-- Packages --{RESET}")
    pkg_checks = [
        Check("dt_teleop_testbed built",      check_package("dt_teleop_testbed")),
        Check("xarm_api built",               check_package("xarm_api")),
        Check("xarm_msgs built",              check_package("xarm_msgs")),
        Check("ros_tcp_endpoint built",       check_package("ros_tcp_endpoint")),
        Check("dt_teleop_testbed executables",
              check_executables("dt_teleop_testbed", [
                  "headless_twin", "mock_lite6", "aoi_logger",
                  "network_conditioner", "twin_to_lite6_bridge",
                  "experiment_runner",
              ])),
    ]
    results.extend(c.run() for c in pkg_checks)

    print(f"\n{BOLD}-- Network & tc --{RESET}")
    net_checks = [
        Check("tc netem privileges",   check_tc),
        Check("ethernet interface",    check_iface(args.iface)),
        Check("output directory",      check_output_dir),
    ]
    results.extend(c.run() for c in net_checks)

    if args.robot_ip:
        print(f"\n{BOLD}-- Robot connectivity --{RESET}")
        robot_checks = [
            Check(f"ping {args.robot_ip}",     check_ping(args.robot_ip)),
            Check("UFACTORY Studio reachable", check_robot_studio(args.robot_ip)),
        ]
        results.extend(c.run() for c in robot_checks)

    print()
    n_total = len(results)
    n_pass = sum(1 for r in results if r)
    if n_pass == n_total:
        print(f"{GREEN}{BOLD}All {n_total} checks passed. Ready to launch.{RESET}")
        return 0
    else:
        n_fail = n_total - n_pass
        print(f"{RED}{BOLD}{n_fail}/{n_total} check(s) failed.{RESET} Fix above before launching.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
