"""Metrics, summaries and tables of the evaluation: a verbatim copy of the pure-python part of
unitree_rl_lab/scripts/rsl_rl/evaluate.py (same time series channels, same contact events, same sprint metrics,
same summary keys, same tables), so that a MuJoCo report and an IsaacLab report are computed in exactly the
same way and can be opened by the same viewer (eval_report.py).

If evaluate.py changes, copy the block from "def _mean" to "##\n# Arguments" (without _drop_args and
_default_report_path) again.
"""

from __future__ import annotations

import fnmatch
import json
import re
import statistics
import warnings
from collections import Counter

import numpy as np


def _mean(values):
    values = [v for v in values if v is not None]
    return statistics.mean(values) if values else None


# Channels of the recorded time series (columns of the per-step rows).
SERIES_CHANNELS = (
    "vx", "vy", "wz",  # measured base velocity, body frame (m/s, m/s, rad/s)
    "cmd_vx", "cmd_vy", "cmd_wz",  # commanded velocity
    "x", "y",  # position relative to the terrain tile centre (m)
    "yaw_err_deg", "tilt_deg", "power_w",
    "contact_force_n",  # largest trunk (base or head) contact force during the step (N)
    "leg_force_n",  # largest leg (hip / thigh / calf) contact force during the step (N)
    "contact",  # 1 while the trunk (base or head) is in contact
    "base_contact", "head_contact", "leg_contact",  # 1 while that part is in contact
)  # fmt: skip
N_KIN = SERIES_CHANNELS.index("contact_force_n")  # the first N_KIN channels are kinematics
FORCE_COL, LEG_FORCE_COL = SERIES_CHANNELS.index("contact_force_n"), SERIES_CHANNELS.index("leg_force_n")
CONTACT_COL = SERIES_CHANNELS.index("contact")
BASE_COL, HEAD_COL, LEG_COL = (SERIES_CHANNELS.index(c) for c in ("base_contact", "head_contact", "leg_contact"))
MAX_POOLED = [FORCE_COL, LEG_FORCE_COL, CONTACT_COL, BASE_COL, HEAD_COL, LEG_COL]  # never averaged away
X_COL = SERIES_CHANNELS.index("x")
CONTACT_MERGE_GAP_S = 0.1  # contacts closer than this count as one event
STOP_SPEED_MPS, STOP_HOLD_S = 0.1, 0.5  # "stopped" = |speed| below this for this long
SPRINT_KEYS = (
    "peak_speed_mps", "time_to_90pct_s", "speed_at_brake_mps", "stop_time_s", "stop_distance_m",
    "min_speed_after_brake_mps",
)  # fmt: skip
TRUNK_KEYS = (
    "trunk_contact_events", "trunk_contact_time_s", "first_trunk_contact_s",
    "max_trunk_contact_force_n", "trunk_contact_intervals_s",
    "base_contact_events", "base_contact_intervals_s",
)  # fmt: skip
HEAD_KEYS = ("head_contact_events", "head_contact_intervals_s")
LEG_KEYS = (
    "leg_contact_events", "leg_contact_time_s", "first_leg_contact_s", "max_leg_contact_force_n",
    "leg_contact_intervals_s",
)  # fmt: skip
CONTACT_KEYS = TRUNK_KEYS + HEAD_KEYS + LEG_KEYS


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """[start, end) index pairs of the runs of True in a boolean array."""
    d = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1).tolist(), np.flatnonzero(d == -1).tolist()))


def contact_intervals(mask: np.ndarray, dt: float, merge_gap_s: float = CONTACT_MERGE_GAP_S) -> list[list[float]]:
    """Time intervals (s) in which mask (one value per control step) is True; close ones are merged."""
    gap = int(round(merge_gap_s / dt))
    merged: list[list[int]] = []
    for start, end in _runs(mask):
        if merged and start - merged[-1][1] <= gap:
            merged[-1][1] = end
        else:
            merged.append([start, end])
    return [[start * dt, end * dt] for start, end in merged]


def sprint_metrics(arr: np.ndarray, dt: float, brake_t: float, v_cmd: float) -> dict:
    """Metrics of "accelerate to v_cmd, command 0 at brake_t" from the full-rate rows (see SERIES_CHANNELS).

    peak speed and time to 90 % of v_cmd (up to the stop command), the speed at the stop command and, after it,
    the stop time (|speed| < STOP_SPEED_MPS for STOP_HOLD_S), the distance travelled until then and the lowest
    speed afterwards (negative = the robot rolled back). Keys are None when the episode ended too early.
    """
    out = dict.fromkeys(SPRINT_KEYS)
    vx, x = arr[:, 0], arr[:, X_COL]
    n = len(arr)
    t = (np.arange(n) + 1) * dt
    run = np.isfinite(vx) & (t <= brake_t + 1e-9)
    if run.any():
        out["peak_speed_mps"] = float(vx[run].max())
        hit = np.flatnonzero(run & (vx >= 0.9 * v_cmd))
        out["time_to_90pct_s"] = float(t[hit[0]]) if hit.size else None
    kb = int(round(brake_t / dt))  # index of the first step that is driven by the stop command
    if kb < 1 or n <= kb:
        return out
    out["speed_at_brake_mps"] = float(vx[kb - 1]) if np.isfinite(vx[kb - 1]) else None
    after = vx[kb:]
    if np.isfinite(after).any():
        out["min_speed_after_brake_mps"] = float(np.nanmin(after))
    hold = max(1, int(round(STOP_HOLD_S / dt)))
    slow = np.isfinite(after) & (np.abs(after) < STOP_SPEED_MPS)
    if slow.size >= hold:
        csum = np.concatenate(([0], np.cumsum(slow)))
        starts = np.flatnonzero(csum[hold:] - csum[:-hold] == hold)  # first window of `hold` slow steps
        if starts.size:
            j = int(starts[0])
            out["stop_time_s"] = j * dt
            if np.isfinite(x[kb + j]) and np.isfinite(x[kb - 1]):
                out["stop_distance_m"] = float(x[kb + j] - x[kb - 1])
    return out


def _jsonable(column: np.ndarray, digits: int = 4) -> list:
    return [round(float(v), digits) if np.isfinite(v) else None for v in column]


def finalize_episode(
    rows: list, dt: float, record_dt: float, brake_time_s: float | None = None, v_cmd: float | None = None
) -> tuple[dict, dict]:
    """Turn the per-step rows of one finished episode into (time series for the report, metrics).

    The metrics are the body-contact metrics (trunk = base or head, per part, legs) and, if brake_time_s
    is given (sprint with a sudden stop), the sprint/brake metrics.

    The series is down-sampled to record_dt: block means, except the contact forces and flags, which use
    the block maximum so that a short contact is never lost. The contact metrics are computed from the
    full-rate data.
    """
    arr = np.asarray(rows, dtype=np.float64).reshape(len(rows), len(SERIES_CHANNELS))
    steps = len(arr)

    def rounded(intervals):
        return [[round(a, 3), round(b, 3)] for a, b in intervals]

    def group(flag_col: int, force_col: int | None):
        flag = arr[:, flag_col] > 0.5
        intervals = contact_intervals(flag, dt)
        force = arr[flag & np.isfinite(arr[:, force_col]), force_col] if force_col is not None else np.empty(0)
        return flag, intervals, force

    trunk, trunk_iv, trunk_f = group(CONTACT_COL, FORCE_COL)
    legs, leg_iv, leg_f = group(LEG_COL, LEG_FORCE_COL)
    _, base_iv, _ = group(BASE_COL, None)
    _, head_iv, _ = group(HEAD_COL, None)
    metrics = {
        "trunk_contact_events": len(trunk_iv),
        "trunk_contact_time_s": float(trunk.sum() * dt),
        "first_trunk_contact_s": round(trunk_iv[0][0], 3) if trunk_iv else None,
        "max_trunk_contact_force_n": float(trunk_f.max()) if trunk_f.size else None,
        "trunk_contact_intervals_s": rounded(trunk_iv),
        "base_contact_events": len(base_iv),
        "base_contact_intervals_s": rounded(base_iv),
        "head_contact_events": len(head_iv),
        "head_contact_intervals_s": rounded(head_iv),
        "leg_contact_events": len(leg_iv),
        "leg_contact_time_s": float(legs.sum() * dt),
        "first_leg_contact_s": round(leg_iv[0][0], 3) if leg_iv else None,
        "max_leg_contact_force_n": float(leg_f.max()) if leg_f.size else None,
        "leg_contact_intervals_s": rounded(leg_iv),
    }
    if brake_time_s is not None and v_cmd:
        metrics.update(sprint_metrics(arr, dt, brake_time_s, v_cmd))

    k = max(1, int(round(record_dt / dt)))
    blocks = -(-steps // k)
    if blocks * k > steps:
        arr = np.vstack([arr, np.full((blocks * k - steps, arr.shape[1]), np.nan)])
    grouped = arr.reshape(blocks, k, -1)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")  # blocks without valid data
        reduced = np.nanmean(grouped, axis=1)
        biggest = np.nanmax(grouped, axis=1)
    reduced[:, MAX_POOLED] = biggest[:, MAX_POOLED]
    times = np.minimum((np.arange(blocks) + 1) * k, steps) * dt
    series = {"dt": round(k * dt, 4), "t": _jsonable(times, 3)}
    for name, column in zip(SERIES_CHANNELS, reduced.T):
        series[name] = _jsonable(column)
    return series, metrics


def _f(value, spec: str) -> str:
    return format(value, spec) if value is not None else "-"


def summarize(episodes: list[dict]) -> dict:
    n = len(episodes)
    ok = [e for e in episodes if e["success"]]
    fell = sum(e["fell_before_goal"] for e in episodes)

    def mean_of(key):
        return _mean(e.get(key) for e in episodes)

    def worst_of(key):
        vals = [e[key] for e in episodes if e.get(key) is not None]
        return max(vals) if vals else None

    def rate_of(key):
        """Share of the episodes with at least one event (None if not measured)."""
        known = [e[key] for e in episodes if e.get(key) is not None]
        return sum(v > 0 for v in known) / len(known) if known else None

    term_keys = sorted({k for e in episodes for k in e.get("reward_terms_per_s", {})})
    return {
        "episodes": n,
        # task
        "success_rate": len(ok) / n,
        "midpoint_rate": sum(e["reached_midpoint"] for e in episodes) / n,
        "fall_rate": fell / n,
        "timeout_rate": (n - len(ok) - fell) / n,
        "mean_time_to_goal_s": _mean(e["time_to_goal_s"] for e in ok),
        "mean_progress_m": mean_of("progress_m"),
        # path
        "mean_max_yaw_err_deg": mean_of("max_yaw_err_deg"),
        "worst_max_yaw_err_deg": worst_of("max_yaw_err_deg"),
        "mean_max_lateral_m": mean_of("max_lateral_m"),
        "worst_max_lateral_m": worst_of("max_lateral_m"),
        # tracking
        "mean_vel_err_xy_mps": mean_of("vel_err_xy_mps"),
        "mean_vel_err_yaw_radps": mean_of("vel_err_yaw_radps"),
        "mean_forward_speed_mps": mean_of("forward_speed_mps"),
        # efficiency
        "mean_power_w": mean_of("power_w"),
        "mean_cost_of_transport": mean_of("cost_of_transport"),
        # smoothness
        "mean_action_rate": mean_of("action_rate"),
        "mean_joint_acc": mean_of("joint_acc"),
        # stability
        "mean_tilt_deg": mean_of("mean_tilt_deg"),
        "worst_max_tilt_deg": worst_of("max_tilt_deg"),
        # body contacts (None if not measured): trunk = base or head, and the legs
        "mean_trunk_contacts_per_episode": mean_of("trunk_contact_events"),
        "trunk_contact_episode_rate": rate_of("trunk_contact_events"),
        "mean_trunk_contact_time_s": mean_of("trunk_contact_time_s"),
        "worst_trunk_contact_force_n": worst_of("max_trunk_contact_force_n"),
        "mean_base_contacts_per_episode": mean_of("base_contact_events"),
        "mean_head_contacts_per_episode": mean_of("head_contact_events"),
        "head_contact_episode_rate": rate_of("head_contact_events"),
        "mean_leg_contacts_per_episode": mean_of("leg_contact_events"),
        "leg_contact_episode_rate": rate_of("leg_contact_events"),
        "mean_leg_contact_time_s": mean_of("leg_contact_time_s"),
        "worst_leg_contact_force_n": worst_of("max_leg_contact_force_n"),
        # sprint with a sudden stop (None for the other scenarios)
        "mean_peak_speed_mps": mean_of("peak_speed_mps"),
        "mean_time_to_90pct_s": mean_of("time_to_90pct_s"),
        "mean_speed_at_brake_mps": mean_of("speed_at_brake_mps"),
        "mean_stop_time_s": mean_of("stop_time_s"),
        "mean_stop_distance_m": mean_of("stop_distance_m"),
        "lowest_speed_after_brake_mps": (
            min(e["min_speed_after_brake_mps"] for e in episodes if e.get("min_speed_after_brake_mps") is not None)
            if any(e.get("min_speed_after_brake_mps") is not None for e in episodes) else None
        ),
        "stopped_rate": (
            sum(e.get("stop_time_s") is not None for e in episodes) / n
            if any("peak_speed_mps" in e for e in episodes) else None
        ),
        # training-like
        "mean_total_reward": mean_of("total_reward"),
        "reward_terms_per_s": {k: _mean(e.get("reward_terms_per_s", {}).get(k) for e in episodes) for k in term_keys},
        "end_reasons": dict(Counter(e.get("end_reason", "unknown") for e in episodes)),
    }


def _fp(value, meta: dict) -> str:
    """Progress with its unit: metres along the course, or degrees for a turn on the spot."""
    if value is None:
        return "-"
    return f"{value:.0f} deg" if (meta or {}).get("progress_unit") == "deg" else f"{value:.2f} m"


def print_tables(results: dict):
    w = max(len(k) for k in results) + 2
    line = "=" * (w + 100)

    # --- table 1: task result and path --------------------------------------------------------
    print("\n" + line)
    print(
        f"{'scenario':<{w}}{'episodes':>9}{'success':>9}{'midpoint':>10}{'falls':>7}{'timeouts':>10}"
        f"{'t_goal, s':>11}{'progress, m':>13}{'yaw err, deg':>14}{'lateral, m':>12}"
    )
    print("-" * (w + 100))
    for name, r in results.items():
        s = r["summary"]
        has_milestone = (r.get("meta") or {}).get("midpoint_x", 0.0) is not None  # flat ground / sprint: no milestone
        mid_txt = f"{s['midpoint_rate']:.0%}" if has_milestone else "-"
        print(
            f"{name:<{w}}{s['episodes']:>9}{s['success_rate']:>9.0%}{mid_txt:>10}"
            f"{s['fall_rate']:>7.0%}{s['timeout_rate']:>10.0%}{_f(s['mean_time_to_goal_s'], '.1f'):>11}"
            f"{_fp(s['mean_progress_m'], r.get('meta')):>13}{_f(s['worst_max_yaw_err_deg'], '.1f'):>14}"
            f"{_f(s['worst_max_lateral_m'], '.2f'):>12}"
        )
    print(line)
    print("yaw err / lateral: worst case over episodes (turn on the spot: lateral = drift from the start point)")

    # --- table 2: motion quality ---------------------------------------------------------------
    line = "=" * (w + 139)
    print("\n" + line)
    print(
        f"{'scenario':<{w}}{'vel err xy':>11}{'vel err yaw':>12}{'speed':>8}{'power, W':>10}{'CoT':>7}"
        f"{'act. rate':>11}{'joint acc':>11}{'tilt max':>10}{'reward':>9}{'trunk/ep':>10}{'head/ep':>9}"
        f"{'w/ trunk':>10}{'legs/ep':>9}{'w/ legs':>9}"
    )
    print("-" * (w + 139))
    for name, r in results.items():
        s = r["summary"]
        print(
            f"{name:<{w}}{_f(s['mean_vel_err_xy_mps'], '.3f'):>11}{_f(s['mean_vel_err_yaw_radps'], '.3f'):>12}"
            f"{_f(s['mean_forward_speed_mps'], '.2f'):>8}{_f(s['mean_power_w'], '.0f'):>10}"
            f"{_f(s['mean_cost_of_transport'], '.2f'):>7}{_f(s['mean_action_rate'], '.2f'):>11}"
            f"{_f(s['mean_joint_acc'], '.0f'):>11}{_f(s['worst_max_tilt_deg'], '.0f'):>10}"
            f"{_f(s['mean_total_reward'], '.1f'):>9}{_f(s['mean_trunk_contacts_per_episode'], '.2f'):>10}"
            f"{_f(s.get('mean_head_contacts_per_episode'), '.2f'):>9}{_f(s['trunk_contact_episode_rate'], '.0%'):>10}"
            f"{_f(s.get('mean_leg_contacts_per_episode'), '.2f'):>9}{_f(s.get('leg_contact_episode_rate'), '.0%'):>9}"
        )
    print(line)
    print("vel err: m/s and rad/s | speed: along the commanded direction (m/s) | tilt max: deg, worst case | reward: mean sum per episode")
    print("trunk = base or head | head/ep: head contacts (the nose hitting a step) | legs = hip, thigh, calf (not the feet)")
    print("x/ep: contact events per episode | w/ x: share of the episodes with at least one such contact")

    # --- sprint with a sudden stop ----------------------------------------------------------------
    sprint = {k: r for k, r in results.items() if r["summary"].get("mean_peak_speed_mps") is not None}
    if sprint:
        line = "=" * (w + 87)
        print("\n" + line)
        print(
            f"{'scenario (sprint + stop)':<{w}}{'peak, m/s':>11}{'t to 90%, s':>13}{'at brake':>10}{'stop time':>11}"
            f"{'stop dist':>11}{'lowest':>9}{'stopped':>9}"
        )
        print("-" * (w + 87))
        for name, r in sprint.items():
            s = r["summary"]
            print(
                f"{name:<{w}}{_f(s['mean_peak_speed_mps'], '.2f'):>11}{_f(s['mean_time_to_90pct_s'], '.1f'):>13}"
                f"{_f(s['mean_speed_at_brake_mps'], '.2f'):>10}{_f(s['mean_stop_time_s'], '.2f'):>11}"
                f"{_f(s['mean_stop_distance_m'], '.2f'):>11}{_f(s['lowest_speed_after_brake_mps'], '.2f'):>9}"
                f"{_f(s['stopped_rate'], '.0%'):>9}"
            )
        print(line)
        print("speeds in m/s | stop time: s from the stop command until |v| < 0.1 m/s for 0.5 s | stop dist: m | lowest: min speed after the command (negative = rolled back)")

    # --- end reasons -----------------------------------------------------------------------------
    print("\nEpisode end reasons:")
    for name, r in results.items():
        reasons = ", ".join(f"{k} x{v}" for k, v in sorted(r["summary"]["end_reasons"].items()))
        print(f"  {name:<{w}}{reasons}")
    print(flush=True)


def _dump_json(obj, path: str):
    """Write indented JSON, but keep every list of plain numbers on one line (the time series are long)."""
    text = json.dumps(obj, indent=2)
    text = re.sub(r'\[\s+([^\[\]{}"]*?)\s+\]', lambda m: "[" + re.sub(r"\s+", " ", m.group(1)) + "]", text)
    with open(path, "w") as f:
        f.write(text)


def _tokens(scenarios: str) -> list[str]:
    return [t.strip() for t in scenarios.split(",") if t.strip()]


def _is_pattern(token: str) -> bool:
    return token in ("all", "rough") or any(c in token for c in "*?[")


def _expand(tokens: list[str], registry: list[str]) -> list[str]:
    names: list[str] = []
    for t in tokens:
        pattern = "*" if t == "all" else ("rough_*" if t == "rough" else t)
        matches = [n for n in registry if fnmatch.fnmatchcase(n, pattern)]
        if not matches:
            raise ValueError(f"'{t}' matches no scenario. Available: {registry}")
        names += [m for m in matches if m not in names]
    return names
