"""Evaluation of a policy on the fixed scenarios of unitree_rl_lab, in MuJoCo, through go2_policy_runner.

The MuJoCo counterpart of unitree_rl_lab/scripts/rsl_rl/evaluate.py: the same scenarios (terrain tile, spawn,
goal, command, heading hold, end conditions), the same per-step metrics and the same JSON report, so that the
report viewer (eval_report.py, a copy of the one of unitree_rl_lab) shows both simulators the same way and the
numbers can be compared one to one (sim-to-sim).

What is evaluated is the whole deployment chain: the exported policy (policy.onnx + deploy.yaml) running in
go2_policy_runner (observation/action pipeline, FSM, safety layer), the DDS bridge of unitree_mujoco and the
MuJoCo physics of the Go2 model.

Prerequisites
  1. Export the scenarios and terrains once (and again after a change of eval_scenarios.py / ROUGH_TERRAINS_CFG),
     in unitree_rl_lab (IsaacLab conda env):
         python scripts/rsl_rl/export_mujoco_scenarios.py --headless --eval_seed 42
  2. Build the workspace (go2_policy_runner + go2_eval) and source it.

Examples (from a terminal with ROS 2 and the workspace sourced; the simulator is started with --sim_python, by
default ~/unitree_env/bin/python3 if it exists, else this Python):

    ros2 run go2_eval evaluate --policy_dir ~/unitree-go2-rl/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity \
        --scenarios rough_pyramid_stairs_10cm --viewer
    ros2 run go2_eval evaluate --policy_dir <run dir> --scenarios rough --episodes 3
    ros2 run go2_eval evaluate --gui

Per (scenario, seed) one simulator process is started (one scene); the controller (policy_runner.launch.py) runs
for the whole evaluation. Every episode:
  1. the robot is placed at the spawn pose of the scenario with the initial joint positions of the IsaacLab
     asset (= the "stand" pose of go2_policy_runner), its base is held in place;
  2. the controller is brought to "stand" and given --stand_time seconds;
  3. the base is released and the controller is switched to "walk" at the same moment (t = 0); the command of
     the scenario is published on /cmd_vel every control step (heading hold on the ground-truth yaw, schedule);
  4. the episode ends exactly as in IsaacLab: goal line / full turn, time out, base contact, bad orientation;
     additionally when the controller leaves "walk" by itself (its safety layer).

Differences from IsaacLab that cannot be removed (they are part of what sim-to-sim measures):
  * everything runs in real time on the wall clock (go2_policy_runner does), so one episode takes as long as
    it lasts, and the runs are not bit-for-bit repeatable: use several episodes;
  * the reward terms are not computed (they need the IsaacLab managers): reward fields are empty;
  * contact forces are MuJoCo contact forces of the robot's collision geoms (the head is part of base_link).
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime

import numpy as np

from go2_eval import metrics as M
from go2_eval.scenarios import Catalog, command_at, find_catalog, held_yaw_rate, resolve_command, scenario_meta, wrap_to_pi

_UNSET = -1.0e9
_GRAVITY = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))


##
# Arguments
##


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Evaluate a policy on the fixed scenarios in MuJoCo (go2_policy_runner).")
    ap.add_argument("--catalog", default=None, help="catalog.json of export_mujoco_scenarios.py (default: found upwards from the current dir, or $GO2_EVAL_CATALOG)")
    ap.add_argument("--policy_dir", default="", help="policy for go2_policy_runner (run dir with exported/ + params/, or logs/rsl_rl/<experiment> = latest run)")
    ap.add_argument("--scenarios", default="all", help="Comma-separated names / glob patterns / 'rough' / 'all'.")
    ap.add_argument("--episodes", type=int, default=5, help="Episodes per scenario and seed.")
    ap.add_argument("--eval_seed", default=None, help="Terrain seed(s), comma-separated (default: the first exported seed).")
    ap.add_argument("--output", default=None, help="Path of the JSON report.")
    ap.add_argument("--speed", type=float, default=None, help="Commanded speed in m/s (default: the scenario's own, 0.5).")
    ap.add_argument("--no_contact_termination", action="store_true", help="Do not end the episode on base contact.")
    ap.add_argument("--record_dt", type=float, default=0.1, help="Time step (s) of the time series stored in the report.")
    ap.add_argument("--no_report", action="store_true", help="Do not open the report viewer when the runs are done.")
    ap.add_argument("--report_save", nargs="?", const="", default=None, help="Save the report figures as PNG (optionally into this folder).")
    ap.add_argument("--viewer", action="store_true", help="Show the MuJoCo viewer (one window per scenario run).")
    ap.add_argument("--gui", action="store_true", help="Open a window to configure the run.")
    ap.add_argument("--list", action="store_true", help="List the scenarios of the catalog and exit.")
    # infrastructure
    ap.add_argument("--network_interface", default="lo", help="DDS interface of the simulator and the controller (default: lo).")
    ap.add_argument("--domain_id", type=int, default=0, help="DDS domain of the simulator and the controller.")
    ap.add_argument("--mujoco_dir", default=None, help="unitree_mujoco root (default: the folder above the catalog).")
    ap.add_argument("--sim_python", default=None, help="Python with mujoco + unitree_sdk2py (default: ~/unitree_env/bin/python3 if it exists, else this Python).")
    ap.add_argument("--runner_config", default="", help="go2_policy_runner parameter file (default: its config/go2.yaml).")
    ap.add_argument("--no_launch_runner", action="store_true", help="Do not start go2_policy_runner (it is already running with input:=none).")
    ap.add_argument("--controller", default="policy_controller", help="Name of the controller node.")
    ap.add_argument("--stand_time", type=float, default=3.0, help="Seconds in 'stand' (base held) before each episode.")
    ap.add_argument("--sim_args", default="", help="Extra arguments for eval_sim.py, e.g. '--dt 0.002'.")
    ap.add_argument("--log_dir", default=None, help="Folder for the logs of the simulator and the controller (default: temp).")
    return ap


##
# Helpers
##


def dds_env(interface: str, domain: int) -> dict:
    env = {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "ROS_DOMAIN_ID": str(domain)}
    if interface and interface != "auto":
        env["CYCLONEDDS_URI"] = (
            "<CycloneDDS><Domain><General><Interfaces>"
            f'<NetworkInterface name="{interface}" priority="default" multicast="default"/>'
            "</Interfaces></General></Domain></CycloneDDS>"
        )
    return env


def default_sim_python() -> str:
    venv = os.path.expanduser("~/unitree_env/bin/python3")
    return venv if os.path.isfile(venv) else sys.executable


def resolve_policy_run(policy_dir: str) -> str | None:
    """Run folder the controller will use (same rule as go2_policy_runner: a folder without exported/ -> latest run)."""
    if not policy_dir:
        return None
    p = os.path.abspath(os.path.expanduser(policy_dir))
    if os.path.isfile(os.path.join(p, "exported", "policy.onnx")):
        return p
    runs = sorted(d for d in glob.glob(os.path.join(p, "*")) if os.path.isfile(os.path.join(d, "exported", "policy.onnx")))
    return runs[-1] if runs else None


def check_policy_dir(policy_dir: str) -> str | None:
    """Error message if go2_policy_runner cannot load this policy folder (checked before anything is started)."""
    p = os.path.abspath(os.path.expanduser(policy_dir))
    if not os.path.isdir(p):
        return f"policy folder not found: {p}"
    run = resolve_policy_run(p)
    if run is None:
        return (
            f"{p} has no exported policy (exported/policy.onnx), neither itself nor in a run sub-folder.\n"
            "Export it in unitree_rl_lab (IsaacLab env), e.g.:\n"
            f"  python scripts/rsl_rl/play.py --task Unitree-Go2-Velocity --load_run {os.path.basename(p)} --headless\n"
            "(play.py writes <run>/exported/policy.onnx; params/deploy.yaml is written by train.py)"
        )
    if not os.path.isfile(os.path.join(run, "params", "deploy.yaml")):
        return f"{run}/params/deploy.yaml not found (it is written by train.py and needed by go2_policy_runner)"
    return None


def yaw_of(q) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def tilt_of(q) -> float:
    """Angle between the base z axis and the world z axis: acos(-projected_gravity_b.z)."""
    w, x, y, z = q
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))


def ang_vel_world_z(q, ang_b) -> float:
    w, x, y, z = q
    r20, r21, r22 = 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)
    return r20 * ang_b[0] + r21 * ang_b[1] + r22 * ang_b[2]


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def course_markers(sc, origin) -> list[dict]:
    """Visual ground lines in the viewer: start (white), midpoint (blue), goal (green)."""
    if sc.motion == "turn" or sc.goal_x is None:
        return []
    dx, dy = sc.course_dir
    nx, ny = -dy, dx
    half = 1.0
    out = []
    for value, rgba in ((sc.spawn_x, "1 1 1 0.9"), (sc.midpoint_x, "0.2 0.5 1 0.8"), (sc.goal_x, "0.1 0.9 0.3 0.9")):
        if value is None:
            continue
        cx, cy = origin[0] + value * dx, origin[1] + value * dy
        out.append({"from": [cx - half * nx, cy - half * ny], "to": [cx + half * nx, cy + half * ny], "rgba": rgba, "width": 0.03})
    return out


class Runner:
    """go2_policy_runner launched with ros2 launch (input:=none: the commands come from this evaluation)."""

    def __init__(self, args, env: dict, log_path: str):
        cmd = [
            "ros2", "launch", "go2_policy_runner", "policy_runner.launch.py",
            f"network_interface:={args.network_interface}", f"domain_id:={args.domain_id}",
            f"policy_dir:={os.path.expanduser(args.policy_dir)}", "input:=none",
        ]  # fmt: skip
        if args.runner_config:
            cmd.append(f"config:={os.path.expanduser(args.runner_config)}")
        self.cmd = cmd
        self.log_path = log_path
        self._log = open(log_path, "w")
        self._log.write("$ " + shlex.join(cmd) + "\n\n")
        self._log.flush()
        self.proc = subprocess.Popen(cmd, env={**os.environ, **env}, stdout=self._log, stderr=subprocess.STDOUT, start_new_session=True)

    def alive(self) -> bool:
        return self.proc.poll() is None

    def stop(self):
        if self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout=10)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._log.close()

    def log_tail(self, lines: int = 25) -> str:
        try:
            with open(self.log_path, errors="replace") as f:
                return "".join(f.readlines()[-lines:])
        except OSError:
            return ""


class EvalError(RuntimeError):
    pass


##
# Controller handling
##


def bring_to_stand(ros, sim, reset_msg: dict, stand_time: float, timeout: float = 30.0):
    """Hold the base, bring the controller to 'stand', place the robot at the spawn pose and let it settle."""
    ros.cmd(0.0, 0.0, 0.0)
    sim.command({"cmd": "hold"})
    deadline = time.monotonic() + timeout
    requested = None
    reset_at = None
    while True:
        if time.monotonic() > deadline:
            st = ros.state()
            raise EvalError(
                "the controller did not reach 'stand' (mode: "
                + (f"{st.mode}, lowstate ok: {st.lowstate_ok}, ready: {st.ready}" if st else "no state received")
                + ")"
            )
        st = ros.state()
        if st is None or not st.ready or not st.lowstate_ok:
            time.sleep(0.05)
            continue
        if st.mode != "stand":
            if requested is None or time.monotonic() - requested > 1.0:
                target = "stand" if "stand" in st.allowed_transitions else "passive"
                ros.set_mode(target)
                requested = time.monotonic()
            time.sleep(0.05)
            continue
        if reset_at is None:  # in stand: now teleport the robot to the spawn pose (base held, joints = stand pose)
            sim.command(reset_msg)
            reset_at = time.monotonic()
        if st.mode_time >= stand_time and time.monotonic() - reset_at >= 0.5 and ros.state_age() < 0.5:
            return
        time.sleep(0.05)


##
# One episode (port of the rollout loop of evaluate.py for one environment)
##


def run_episode(sim, ros, sc, ctx: dict, args) -> dict:
    dt = ctx["step_dt"]
    max_steps = math.ceil(sc.episode_length_s / dt)
    ox, oy = ctx["origin"][0], ctx["origin"][1]
    dir_x, dir_y = sc.course_dir
    spawn_px, spawn_py = sc.spawn_x * dir_x, sc.spawn_x * dir_y
    is_turn = sc.motion == "turn"
    rot_sign = -1.0 if sc.wz < 0.0 else 1.0
    thr = ctx["contact_threshold"]
    bad_limit = ctx["bad_orientation"]
    contact_term = ctx["contact_term_name"] if not args.no_contact_termination else None

    # --- spawn (eval_scenarios.reset_root_state_fixed) ---
    yaw_q = [math.cos(sc.heading / 2), 0.0, 0.0, math.sin(sc.heading / 2)]
    reset_msg = {
        "cmd": "reset",
        "pos": [ox + spawn_px, oy + spawn_py, ctx["init_pos"][2]],
        "quat": quat_mul(yaw_q, ctx["init_rot"]),
        "joints": ctx["init_joints"],
    }
    bring_to_stand(ros, sim, reset_msg, args.stand_time)

    # --- start: the command of the first step, release the base and run the policy ---
    cmd = command_at(sc, 0.0)
    ros.cmd(*cmd)
    ros.reset_actions()
    sim.drain()
    sim.command({"cmd": "release"})
    walk_future = ros.set_mode_async("walk")
    t_walk = time.monotonic()

    rows: list = []
    steps = alive_steps = 0
    max_x, reached_mid = _UNSET, False
    max_yaw_err = max_lat = max_tilt = 0.0
    yaw_total, prev_heading, rot_total = 0.0, sc.heading, 0.0
    sums = dict.fromkeys(("vel_err_xy", "vel_err_yaw", "fwd_speed", "power", "action_rate", "joint_acc", "tilt"), 0.0)
    rate_hist, lag = [], {}
    wall_t = 0.0
    while True:
        s = sim.sample(timeout=3.0)
        steps += 1
        quat, ang_b, lin_b = s["quat"], s["ang_vel_b"], s["lin_vel_b"]
        px, py = s["pos"][0] - ox, s["pos"][1] - oy
        heading = yaw_of(quat)
        if is_turn:
            yaw_total += wrap_to_pi(heading - prev_heading)
            prev_heading = heading
            x = math.degrees(yaw_total) * rot_sign
            y = math.hypot(px - spawn_px, py - spawn_py)
            yaw_err = float("nan")
        else:
            x = px * dir_x + py * dir_y
            y = py * dir_x - px * dir_y
            yaw_err = abs(wrap_to_pi(heading - sc.heading))
        vel_err_xy = math.hypot(cmd[0] - lin_b[0], cmd[1] - lin_b[1])
        vel_err_yaw = abs(cmd[2] - ang_b[2])
        if sc.primary_channel == "vx":
            fwd_speed = lin_b[0] * sc.primary_sign
        elif sc.primary_channel == "vy":
            fwd_speed = lin_b[1] * sc.primary_sign
        else:
            fwd_speed = 0.0
        power = s["power"]
        action_rate = ros.take_action_rate()
        joint_acc = s["joint_acc"]
        tilt = tilt_of(quat)
        fb, fh, fl = s["forces"]["base"], s["forces"]["head"], s["forces"]["legs"]
        rot_total += ang_vel_world_z(quat, ang_b) * dt  # eval_scenarios.rotation_reached

        # --- terminations of this step (IsaacLab termination terms) ---
        flags = {"time_out": steps >= max_steps}
        if contact_term:
            flags[contact_term] = fb > thr
        if bad_limit is not None:
            flags["bad_orientation"] = tilt > bad_limit
        if sc.motion == "walk" and sc.goal_x is not None:
            flags["goal_reached"] = x >= sc.goal_x
        elif is_turn:
            flags["goal_reached"] = math.degrees(abs(rot_total)) >= sc.goal_x
        st = ros.state()
        if time.monotonic() - t_walk > 0.5 and (st is None or st.mode != "walk"):
            flags[f"controller_{st.mode if st else 'lost'}"] = True  # the controller's safety layer stopped the policy
        if st is not None:
            rate_hist.append(float(st.lowstate_rate))
        done = any(flags.values())
        timeout = flags["time_out"]
        alive = not done
        goal_hit = flags.get("goal_reached", False)
        term_contact = bool(contact_term and flags.get(contact_term))

        if alive:
            max_x = max(max_x, x)
            if not is_turn:
                max_yaw_err = max(max_yaw_err, yaw_err)
            max_lat = max(max_lat, abs(y))
            max_tilt = max(max_tilt, tilt)
            for key, value in (
                ("vel_err_xy", vel_err_xy), ("vel_err_yaw", vel_err_yaw), ("fwd_speed", fwd_speed), ("power", power),
                ("action_rate", action_rate), ("joint_acc", joint_acc), ("tilt", tilt),
            ):  # fmt: skip
                sums[key] += value
            alive_steps += 1
            if sc.midpoint_x is not None and max_x >= sc.midpoint_x:
                reached_mid = True

        row = np.full(len(M.SERIES_CHANNELS), np.nan)
        row[3:6] = cmd
        if done:  # as in evaluate.py: the state of the terminating step belongs to the next episode there
            row[[M.CONTACT_COL, M.BASE_COL]] = float(term_contact)
            row[[M.HEAD_COL, M.LEG_COL]] = 0.0
        else:
            row[: M.N_KIN] = [lin_b[0], lin_b[1], ang_b[2], cmd[0], cmd[1], cmd[2], x, y,
                              math.degrees(yaw_err), math.degrees(tilt), power]  # fmt: skip
            row[M.FORCE_COL], row[M.LEG_FORCE_COL] = max(fb, fh), fl
            row[M.BASE_COL], row[M.HEAD_COL], row[M.LEG_COL] = float(fb > thr), float(fh > thr), float(fl > thr)
            row[M.CONTACT_COL] = max(row[M.BASE_COL], row[M.HEAD_COL])
        rows.append(row)
        lag, wall_t = s.get("lag", {}), s.get("wall", 0.0)

        if done:
            break
        # --- command of the next step (command_schedule, then hold_heading) ---
        cmd = command_at(sc, steps * dt)
        if sc.hold_heading:
            cmd = (cmd[0], cmd[1], held_yaw_rate(sc, heading))
        ros.cmd(*cmd)

    sim.command({"cmd": "hold"})
    ros.cmd(0.0, 0.0, 0.0)
    try:
        answer = walk_future.result() if walk_future.done() else None
    except Exception as e:  # noqa: BLE001
        answer = None
        print(f"[WARN] set_mode walk: {e}", flush=True)
    if answer is None or not answer.success:
        print(f"[WARN] switching the controller to 'walk' was not confirmed: {answer.message if answer else 'no answer'}", flush=True)

    # --- episode metrics (as evaluate.py) ---
    series, ep_metrics = M.finalize_episode(
        rows, dt, args.record_dt, brake_time_s=sc.brake_time_s if sc.motion == "sprint_brake" else None, v_cmd=sc.vx
    )
    survived_and_stopped = sc.success_on_timeout and timeout and ep_metrics.get("stop_time_s") is not None
    success = bool(goal_hit) or survived_and_stopped
    best_x = max_x if max_x > _UNSET / 2 else sc.spawn_x
    if success and sc.goal_x is not None:
        best_x = max(best_x, sc.goal_x)
    progress = best_x - sc.spawn_x
    duration = steps * dt
    k = max(float(alive_steps), 1.0)
    mean_power = sums["power"] / k
    energy = mean_power * k * dt
    cot = energy / (ctx["mass"] * _GRAVITY * progress) if (not is_turn and progress > 0.5) else None
    reasons = [nm for nm, v in flags.items() if v]
    return {
        "seed": ctx["seed"],
        "success": success,
        "fell_before_goal": bool((not timeout) and (not success)),
        "reached_midpoint": success or (reached_mid if sc.midpoint_x is not None else False),
        "time_to_goal_s": duration if (success and not sc.success_on_timeout) else None,
        "duration_s": duration,
        "progress_m": progress,
        "max_yaw_err_deg": None if is_turn else math.degrees(max_yaw_err),
        "max_lateral_m": max_lat,
        "vel_err_xy_mps": sums["vel_err_xy"] / k,
        "vel_err_yaw_radps": sums["vel_err_yaw"] / k,
        "forward_speed_mps": None if is_turn else sums["fwd_speed"] / k,
        "power_w": mean_power,
        "cost_of_transport": cot,
        "action_rate": sums["action_rate"] / k,
        "joint_acc": sums["joint_acc"] / k,
        "mean_tilt_deg": math.degrees(sums["tilt"] / k),
        "max_tilt_deg": math.degrees(max_tilt),
        "total_reward": None,  # the reward terms need the IsaacLab managers
        "reward_terms_per_s": {},
        "end_reason": "+".join(sorted(reasons)) if reasons else ("time_out" if timeout else "unknown"),
        "command_speed_mps": None if is_turn else sc.primary_command,
        **ep_metrics,
        "mujoco": {
            "real_time_factor": round(duration / wall_t, 3) if wall_t > 0 else None,
            "sim_overruns": lag.get("overruns"),
            "min_lowstate_rate_hz": round(min(rate_hist), 1) if rate_hist else None,
            "policy_steps": ros.policy_steps,
        },
        "series": series,
    }


##
# One (scenario, seed) run
##


def run_pair(name: str, seed: int, catalog: Catalog, ros, args, log_dir: str) -> dict:
    from go2_eval.sim_link import SimError, SimProcess

    task = catalog.task
    sc = resolve_command(catalog.scenario(name), name, args.speed, task["command_limit_ranges"])
    rel = catalog.data["scenarios"][name]["terrains"].get(str(seed))
    if rel is None:
        raise EvalError(f"'{name}' has no terrain for seed {seed} (exported: {catalog.seeds(name)}); export it with --append --eval_seed {seed}")
    origin = list(np.load(os.path.join(catalog.dir, rel))["origin"])
    sim = SimProcess(
        sim_dir=os.path.join(args.mujoco_dir, "simulate_python"),
        python=args.sim_python,
        catalog=catalog.path,
        scenario=name,
        seed=seed,
        domain=args.domain_id,
        interface=args.network_interface,
        sample_dt=task["step_dt_s"],
        markers=course_markers(sc, origin),
        viewer=args.viewer,
        log_path=os.path.join(log_dir, f"sim_{name}_{seed}.log"),
        extra_args=shlex.split(args.sim_args),
    )
    print(f"[INFO] [{name}] starting the simulator (log: {sim.log_path}) ...", flush=True)
    try:
        hello = sim.start()
    except SimError as e:
        raise EvalError(f"{e}\n--- simulator log ({sim.log_path}) ---\n{sim.log_tail()}") from None

    terms = catalog.contact_terms()
    ctx = {
        "seed": seed,
        "step_dt": task["step_dt_s"],
        "origin": origin,
        "init_pos": task["robot"]["init_pos"],
        "init_rot": task["robot"]["init_rot"],
        "init_joints": catalog.joint_positions(hello["joint_names"]),
        "contact_threshold": catalog.contact_threshold(),
        "contact_term_name": terms[0]["term"] if terms else "base_contact",
        "bad_orientation": catalog.bad_orientation(),
        "mass": hello["mass"],
    }
    if len(ctx["init_joints"]) != len(hello["joint_names"]):
        print(f"[WARN] initial position unknown for some joints: {set(hello['joint_names']) - set(ctx['init_joints'])}")

    goal_txt = "none (fixed duration)" if sc.goal_x is None else (f"{sc.goal_x:.0f} deg" if sc.motion == "turn" else f"{sc.goal_x:.2f} m")
    if sc.motion == "sprint_brake":
        command_txt = f"0 -> {sc.vx:.2f} m/s (maximum of the training range), stop command at {sc.brake_time_s:.0f} s"
    elif sc.motion == "turn":
        command_txt = f"{sc.wz:.2f} rad/s ({sc.speed_label})"
    else:
        command_txt = f"{sc.primary_command:.2f} m/s ({sc.speed_label})"
    print(f"[INFO] [{name}] seed {seed}: {sc.description}")
    print(f"[INFO] [{name}] terrain: {hello['terrain']['kind']} ({hello['terrain']['sub_terrain']}, d={hello['terrain']['difficulty']:.3f}); "
          f"spawn {sc.spawn_x:.2f} {sc.progress_unit}, goal {goal_txt}")  # fmt: skip
    print(f"[INFO] [{name}] command: {command_txt}; running {args.episodes} episodes, up to {sc.episode_length_s:.0f} s each", flush=True)

    episodes = []
    try:
        for i in range(args.episodes):
            try:
                ep = run_episode(sim, ros, sc, ctx, args)
            except SimError as e:
                raise EvalError(f"{e}\n--- simulator log ({sim.log_path}) ---\n{sim.log_tail()}") from None
            episodes.append(ep)
            verdict = "SUCCESS" if ep["success"] else ("FELL" if ep["fell_before_goal"] else "TIMEOUT")
            mid = f" {sc.midpoint_label}={'yes' if ep['reached_midpoint'] else 'no'}" if sc.midpoint_x is not None else ""
            prog = f"{ep['progress_m']:.0f}deg" if sc.motion == "turn" else f"{ep['progress_m']:.2f}m"
            sprint = ""
            if ep.get("peak_speed_mps") is not None:
                stop = ep.get("stop_time_s")
                sprint = f" peak={ep['peak_speed_mps']:.2f}m/s stop_time=" + (f"{stop:.2f}s" if stop is not None else "not stopped")
            mj = ep["mujoco"]
            print(
                f"[{name}] episode {i + 1}/{args.episodes}: {verdict:<8} time={ep['duration_s']:.1f}s progress={prog}{mid}{sprint} "
                f"yaw_err={M._f(ep['max_yaw_err_deg'], '.1f')}deg lateral={ep['max_lateral_m']:.2f}m vel_err={ep['vel_err_xy_mps']:.2f}m/s "
                f"contacts: trunk={ep['trunk_contact_events']} (head {ep['head_contact_events']}) legs={ep['leg_contact_events']} "
                f"end={ep['end_reason']} rtf={M._f(mj['real_time_factor'], '.2f')}",
                flush=True,
            )
    finally:
        sim.close()

    meta = {
        **scenario_meta(sc),
        "control_dt_s": task["step_dt_s"],
        "record_dt_s": episodes[0]["series"]["dt"] if episodes else args.record_dt,
        "trunk_contact": {
            "sensor": "mujoco contacts (collision geoms vs. terrain)",
            "bodies": hello["parts"]["base"] + hello["parts"]["head"],
            "parts": {p: hello["parts"][p] for p in ("base", "head", "legs")},
            "threshold_n": ctx["contact_threshold"],
            "terminates_episode": not args.no_contact_termination,
            "merge_gap_s": M.CONTACT_MERGE_GAP_S,
        },
        "terrain": hello["terrain"],
    }
    return {"description": sc.description, "meta": meta, "episodes": episodes, "hello": hello}


##
# Main
##


def _plots_dir(report_path: str, report_save: str) -> str:
    if report_save and report_save.strip():
        return os.path.abspath(os.path.expanduser(report_save.strip()))
    return os.path.splitext(os.path.abspath(report_path))[0]


def open_report(report_path: str, args):
    cmd = [sys.executable, "-m", "go2_eval.eval_report", report_path]
    if args.report_save is not None:
        plots = _plots_dir(report_path, args.report_save)
        cmd += ["--save", plots]
        print(f"[INFO] Figures will be saved to: {plots}", flush=True)
    if args.no_report:
        cmd.append("--no_show")
    else:
        print("[INFO] Opening the report viewer (close its windows to finish) ...", flush=True)
    if subprocess.run(cmd).returncode != 0:
        print(f"[WARN] The report viewer failed (is matplotlib installed?). View it later with:\n  ros2 run go2_eval eval_report {report_path}")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.catalog is None:
        args.catalog = find_catalog()
    if args.gui:
        from go2_eval import eval_gui

        gui_argv = eval_gui.run_gui(args.catalog)
        if gui_argv is None:
            print("[INFO] Cancelled.")
            return 0
        args = parser.parse_args(gui_argv)
        if args.catalog is None:
            args.catalog = find_catalog()
        print("[INFO] Equivalent command:\n  ros2 run go2_eval evaluate " + shlex.join(gui_argv), flush=True)

    if not args.catalog or not os.path.isfile(os.path.expanduser(args.catalog)):
        parser.error(
            "scenario catalog not found. Export it in unitree_rl_lab first:\n"
            "  python scripts/rsl_rl/export_mujoco_scenarios.py --headless\n"
            "and pass --catalog <unitree_mujoco>/eval_scenarios/catalog.json (or run from inside unitree-go2-rl)."
        )
    catalog = Catalog(args.catalog)
    if args.list:
        for name, desc in catalog.descriptions().items():
            print(f"{name:<36} seeds {catalog.seeds(name)}  {desc}")
        return 0
    if args.episodes < 1:
        parser.error("--episodes must be at least 1")
    if args.speed is not None and args.speed <= 0:
        parser.error("--speed must be positive")
    if not args.policy_dir and not args.no_launch_runner:
        parser.error("--policy_dir is required (or --no_launch_runner with a running controller)")

    if not args.no_launch_runner:
        problem = check_policy_dir(args.policy_dir)
        if problem:
            parser.error(problem)

    args.mujoco_dir = os.path.abspath(os.path.expanduser(args.mujoco_dir or os.path.join(catalog.dir, "..")))
    if not os.path.isfile(os.path.join(args.mujoco_dir, "simulate_python", "eval_sim.py")):
        parser.error(f"eval_sim.py not found in {args.mujoco_dir}/simulate_python (set --mujoco_dir)")
    args.sim_python = os.path.expanduser(args.sim_python or default_sim_python())

    try:
        names = M._expand(M._tokens(args.scenarios), catalog.names)
    except ValueError as e:
        parser.error(str(e))
    if args.eval_seed:
        seeds = [int(s) for s in args.eval_seed.split(",") if s.strip()]
    else:
        seeds = [catalog.seeds(names[0])[0]]
    missing = [(n, s) for n in names for s in seeds if str(s) not in catalog.data["scenarios"][n]["terrains"]]
    if missing:
        parser.error(f"no exported terrain for {missing[:5]}{' ...' if len(missing) > 5 else ''}; export them with --append --eval_seed")

    log_dir = os.path.abspath(os.path.expanduser(args.log_dir)) if args.log_dir else tempfile.mkdtemp(prefix="go2_eval_")
    os.makedirs(log_dir, exist_ok=True)
    env = dds_env(args.network_interface, args.domain_id)
    os.environ.update(env)  # before rclpy is imported: the evaluation node joins the controller's DDS domain

    run_dir = resolve_policy_run(args.policy_dir)
    total = len(names) * len(seeds)
    print(f"[INFO] {len(names)} scenario(s) x {len(seeds)} seed(s) = {total} run(s): {names}", flush=True)
    print(f"[INFO] policy: {run_dir or args.policy_dir or '(controller already running)'}; logs: {log_dir}", flush=True)

    from go2_eval.ros_link import RunnerLink

    runner = None if args.no_launch_runner else Runner(args, env, os.path.join(log_dir, "policy_runner.log"))
    ros = RunnerLink(controller=args.controller)
    pooled: dict = {}
    failed = 0
    hello = None
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    try:
        where = f" (log: {runner.log_path})" if runner else ""
        print(f"[INFO] Waiting for go2_policy_runner{where} ...", flush=True)
        up = False
        for _ in range(30):
            if ros.wait_for_service(1.0):
                up = True
                break
            if runner is not None and not runner.alive():
                break
        if not up:
            tail = runner.log_tail() if runner else ""
            state = "exited" if runner is not None and not runner.alive() else "is not available"
            raise EvalError(f"go2_policy_runner {state} (service /{args.controller}/set_mode).\n{tail}")
        print("[INFO] go2_policy_runner is up.", flush=True)
        done_runs = 0
        for name in names:
            for seed in seeds:
                done_runs += 1
                print(f"\n[INFO] ===== run {done_runs}/{total}: {name}, seed {seed} =====", flush=True)
                if runner is not None and not runner.alive():
                    raise EvalError(f"go2_policy_runner exited:\n{runner.log_tail()}")
                try:
                    res = run_pair(name, seed, catalog, ros, args, log_dir)
                except EvalError as e:
                    print(f"[ERROR] '{name}' (seed {seed}) failed: {e}\nContinuing.", flush=True)
                    failed += 1
                    continue
                hello = hello or res["hello"]
                slot = pooled.setdefault(name, {"description": res["description"], "meta": res["meta"], "seeds": [], "episodes": []})
                slot["seeds"].append(seed)
                slot["episodes"] += res["episodes"]
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted: writing the report of the finished runs.", flush=True)
    except EvalError as e:
        print(f"[ERROR] {e}", flush=True)
    finally:
        ros.shutdown()
        if runner is not None:
            runner.stop()

    pooled = {k: v for k, v in pooled.items() if v["episodes"]}
    if not pooled:
        print("[ERROR] No run finished.")
        return 1
    results = {}
    for name, slot in pooled.items():
        slot["summary"] = M.summarize(slot["episodes"])
        results[name] = slot
    M.print_tables(results)

    if args.output:
        out = os.path.abspath(os.path.expanduser(args.output))
    else:
        base = os.path.join(run_dir, "eval") if run_dir else os.path.join(os.getcwd(), "eval")
        out = os.path.join(base, f"mujoco_eval_{stamp}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    report = {
        # the viewer shows "run" = folder of the checkpoint and "model" = its file name
        "checkpoint": os.path.join(run_dir, "policy.onnx") if run_dir else (args.policy_dir or "?"),
        "task": f"MuJoCo | {catalog.task.get('task', '?')}",
        "simulator": "mujoco",
        "num_envs": 1,
        "seeds": seeds,
        "speed_override_mps": args.speed,
        "date": stamp,
        "policy_dir": os.path.abspath(os.path.expanduser(args.policy_dir)) if args.policy_dir else None,
        "catalog": catalog.path,
        "catalog_created": catalog.data.get("created"),
        "mujoco": {
            "version": (hello or {}).get("mujoco"),
            "physics_dt_s": (hello or {}).get("dt"),
            "network_interface": args.network_interface,
            "domain_id": args.domain_id,
            "stand_time_s": args.stand_time,
            "contact_termination": not args.no_contact_termination,
        },
        "scenarios": results,
    }
    M._dump_json(report, out)
    print(f"[INFO] Report saved to: {out}")
    if failed:
        print(f"[WARN] {failed} run(s) failed, their episodes are missing from the report (logs: {log_dir}).")
    if not args.no_report or args.report_save is not None:
        open_report(out, args)
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())