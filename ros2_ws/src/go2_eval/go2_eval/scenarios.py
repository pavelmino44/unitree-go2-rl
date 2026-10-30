"""Scenarios of the MuJoCo evaluation, read from the catalog written by
unitree_rl_lab/scripts/rsl_rl/export_mujoco_scenarios.py.

The EvalScenario of eval_scenarios.py is rebuilt from the exported fields, and the parts of
eval_scenarios.py that the evaluation needs at run time (_resolve_command, scenario_meta, episode_length_s,
the heading hold and the command schedule) are ported 1:1, so that --speed and the training command ranges
act exactly as in evaluate.py.
"""

from __future__ import annotations

import dataclasses
import json
import math
import os
import re
from dataclasses import dataclass

# Heading hold of eval_scenarios.py: wz = clip(HEADING_STIFFNESS * (heading - yaw), +-MAX_YAW_RATE)
HEADING_STIFFNESS = 0.5
MAX_YAW_RATE = 1.0


@dataclass(frozen=True)
class EvalScenario:
    """Same fields as unitree_rl_lab eval_scenarios.EvalScenario (without the terrain config object)."""

    description: str
    size: float
    vx: float
    spawn_x: float
    goal_x: float | None
    vy: float = 0.0
    wz: float = 0.0
    heading: float = 0.0
    hold_heading: bool = True
    difficulty: float = 0.0
    midpoint_x: float | None = None
    midpoint_label: str = "top"
    obstacle_half_width: float | None = None
    timeout_margin_s: float = 5.0
    motion: str = "walk"
    course_dir: tuple = (1.0, 0.0)
    speed_label: str = "forward speed"
    command_schedule: tuple | None = None
    fixed_episode_length_s: float | None = None
    brake_time_s: float | None = None
    success_on_timeout: bool = False

    @property
    def primary_channel(self) -> str:
        if self.motion == "turn":
            return "wz"
        return "vy" if abs(self.vy) > abs(self.vx) else "vx"

    @property
    def primary_sign(self) -> float:
        return -1.0 if getattr(self, self.primary_channel) < 0.0 else 1.0

    @property
    def primary_command(self) -> float:
        return abs(getattr(self, self.primary_channel))

    @property
    def primary_unit(self) -> str:
        return "rad/s" if self.motion == "turn" else "m/s"

    @property
    def progress_unit(self) -> str:
        return "deg" if self.motion == "turn" else "m"

    @property
    def episode_length_s(self) -> float:
        if self.fixed_episode_length_s is not None:
            return self.fixed_episode_length_s
        distance = abs(self.goal_x - self.spawn_x)
        if self.motion == "turn":
            ideal = math.radians(distance) / max(abs(self.wz), 1e-3)
        else:
            ideal = distance / max(math.hypot(self.vx, self.vy), 1e-3)
        return 2.0 * ideal + self.timeout_margin_s

    @classmethod
    def from_fields(cls, fields: dict) -> "EvalScenario":
        names = {f.name for f in dataclasses.fields(cls)}
        kw = {k: v for k, v in fields.items() if k in names}
        if kw.get("course_dir") is not None:
            kw["course_dir"] = tuple(kw["course_dir"])
        if kw.get("command_schedule") is not None:
            kw["command_schedule"] = tuple(tuple(s) for s in kw["command_schedule"])
        return cls(**kw)


def scenario_meta(sc: EvalScenario) -> dict:
    """Port of eval_scenarios.scenario_meta (the data the report viewer needs)."""
    return {
        "description": sc.description,
        "motion": sc.motion,
        "course_dir": list(sc.course_dir),
        "command_speed_mps": None if sc.motion == "turn" else sc.primary_command,
        "primary_channel": sc.primary_channel,
        "primary_sign": sc.primary_sign,
        "primary_label": sc.speed_label,
        "primary_unit": sc.primary_unit,
        "primary_command": sc.primary_command,
        "progress_unit": sc.progress_unit,
        "brake_time_s": sc.brake_time_s,
        "command_schedule": [list(segment) for segment in sc.command_schedule] if sc.command_schedule else None,
        "spawn_x": sc.spawn_x,
        "goal_x": sc.goal_x,
        "midpoint_x": sc.midpoint_x,
        "midpoint_label": sc.midpoint_label,
        "obstacle_half_width": sc.obstacle_half_width,
    }


def resolve_command(sc: EvalScenario, name: str, speed: float | None, limits: dict) -> EvalScenario:
    """Port of eval_scenarios._resolve_command: apply --speed and the maximum of the training range (sprint)."""
    if speed is not None and speed <= 0.0:
        raise ValueError(f"The commanded speed must be positive, got {speed}.")
    lim_x, lim_y, lim_w = tuple(limits["lin_vel_x"]), tuple(limits["lin_vel_y"]), tuple(limits["ang_vel_z"])

    def check(label: str, value: float, lim: tuple, unit: str):
        if not lim[0] <= value <= lim[1]:
            print(f"[WARN] '{name}': {label} {value} {unit} is outside the training range [{lim[0]}, {lim[1]}] {unit}.", flush=True)

    if sc.motion == "sprint_brake":
        vmax = float(lim_x[1])
        if speed is not None:
            print(f"[INFO] --speed is ignored by '{name}': it always runs up to {vmax} m/s, the maximum of the training range.", flush=True)
        schedule = ((0.0, vmax, 0.0, 0.0), (float(sc.brake_time_s), 0.0, 0.0, 0.0))
        return dataclasses.replace(sc, vx=vmax, command_schedule=schedule)
    if sc.motion == "turn":
        if speed is not None:
            print(f"[INFO] --speed is ignored by '{name}': it turns at {sc.wz} rad/s.", flush=True)
        check("turn rate", sc.wz, lim_w, "rad/s")
        return sc
    if speed is not None:
        sc = dataclasses.replace(sc, **{sc.primary_channel: sc.primary_sign * float(speed)})
    check("speed", sc.vx, lim_x, "m/s")
    check("sideways speed", sc.vy, lim_y, "m/s")
    return sc


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def command_at(sc: EvalScenario, time_s: float) -> tuple[float, float, float]:
    """Command of the schedule at time_s (eval_scenarios.command_schedule), or the constant command."""
    if not sc.command_schedule:
        return sc.vx, sc.vy, sc.wz
    out = (0.0, 0.0, 0.0)
    for start_s, vx, vy, wz in sc.command_schedule:
        if time_s >= start_s - 1e-6:
            out = (vx, vy, wz)
    return out


def held_yaw_rate(sc: EvalScenario, heading: float) -> float:
    """eval_scenarios.hold_heading."""
    err = wrap_to_pi(sc.heading - heading)
    return max(-MAX_YAW_RATE, min(MAX_YAW_RATE, HEADING_STIFFNESS * err))


##
# Catalog
##


class Catalog:
    def __init__(self, path: str):
        self.path = os.path.abspath(os.path.expanduser(path))
        with open(self.path, encoding="utf-8") as f:
            self.data = json.load(f)
        self.dir = os.path.dirname(self.path)
        self.task = self.data["task"]

    @property
    def names(self) -> list[str]:
        return list(self.data["scenarios"])

    def descriptions(self) -> dict:
        return {k: v["description"] for k, v in self.data["scenarios"].items()}

    def seeds(self, name: str) -> list[int]:
        return sorted(int(s) for s in self.data["scenarios"][name]["terrains"])

    def scenario(self, name: str) -> EvalScenario:
        return EvalScenario.from_fields(self.data["scenarios"][name]["fields"])

    def joint_positions(self, joint_names: list[str]) -> dict:
        """Initial joint positions of the robot (regex dict of the IsaacLab asset) for the MuJoCo joints."""
        table = self.task["robot"]["init_joint_pos"]
        out = {}
        for joint in joint_names:
            for pattern, value in table.items():
                if re.fullmatch(pattern, joint):
                    out[joint] = float(value)
                    break
        return out

    def contact_threshold(self) -> float:
        terms = self.task["terminations"].get("trunk_contact") or []
        return float(terms[0]["threshold"]) if terms else 1.0

    def contact_terms(self) -> list[dict]:
        return list(self.task["terminations"].get("trunk_contact") or [])

    def bad_orientation(self) -> float | None:
        term = self.task["terminations"].get("bad_orientation")
        return float(term["limit_angle"]) if term else None


def find_catalog(start: str | None = None) -> str | None:
    """Look for unitree_mujoco/eval_scenarios/catalog.json from the current directory upwards
    (unitree-go2-rl layout: ros2_ws/ and unitree_mujoco/ are siblings)."""
    env = os.environ.get("GO2_EVAL_CATALOG")
    if env and os.path.isfile(env):
        return os.path.abspath(env)
    here = os.path.abspath(start or os.getcwd())
    while True:
        for rel in (("unitree_mujoco", "eval_scenarios", "catalog.json"), ("eval_scenarios", "catalog.json")):
            p = os.path.join(here, *rel)
            if os.path.isfile(p):
                return p
        parent = os.path.dirname(here)
        if parent == here:
            return None
        here = parent
