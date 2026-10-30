"""Sim-to-sim comparison: an IsaacLab report (unitree_rl_lab evaluate.py) against a MuJoCo report (go2_eval).

Both reports have the same format, so the scenarios are matched by name and the same summary numbers are
put side by side:

    ros2 run go2_eval eval_compare isaac_report.json mujoco_report.json
    ros2 run go2_eval eval_compare isaac.json mujoco.json --save                  # + save into the default folder
    ros2 run go2_eval eval_compare isaac.json mujoco.json --scenarios "rough_pyramid_stairs_*cm" --save plots/

Output: a table in the terminal (IsaacLab | MuJoCo | difference per metric) and two figures:
  * sim_to_sim: per scenario, vertical bars IsaacLab / MuJoCo (and every episode as a dot) for success rate,
    speed / command, mean power and trunk contacts (2 x 2, left to right), and the capability sweeps (stairs height, slope angle,
    random blocks) of both;
  * metrics: one panel per metric of "Mean over the common scenarios" (the two means as bars, every episode
    as a dot, the difference MuJoCo - IsaacLab: green = better, red = worse than IsaacLab). Each report can still be opened with eval_report.py for
the per-scenario and per-episode windows, which look the same for both simulators.

--save [DIR] writes the figures (sim_to_sim.png, metrics.png) and everything printed in the terminal (compare.txt) into DIR.
Without DIR: next to the MuJoCo report (the eval/ folder of the policy run), in sim-to-sim_compare_<date>/.
Without a display the results are always saved (to that default folder unless DIR is given).
"""

from __future__ import annotations

import argparse
import contextlib
import fnmatch
import io
import json
import os
import shlex
import sys
from datetime import datetime

import numpy as np

from go2_eval import eval_report as R

C_ISAAC = "#2c6db4"
C_MUJOCO = "#e08a1e"

# (label, summary key, format, "higher is better" (True/False/None), per-episode key)
ROWS = (
    ("success", "success_rate", ".0%", True, "success"),
    ("falls", "fall_rate", ".0%", False, "fell_before_goal"),
    ("time to goal, s", "mean_time_to_goal_s", ".1f", False, "time_to_goal_s"),
    ("speed, m/s", "mean_forward_speed_mps", ".2f", None, "forward_speed_mps"),
    ("vel err xy, m/s", "mean_vel_err_xy_mps", ".3f", False, "vel_err_xy_mps"),
    ("vel err yaw, rad/s", "mean_vel_err_yaw_radps", ".3f", False, "vel_err_yaw_radps"),
    ("yaw err max, deg", "worst_max_yaw_err_deg", ".1f", False, "max_yaw_err_deg"),
    ("lateral max, m", "worst_max_lateral_m", ".2f", False, "max_lateral_m"),
    ("power, W", "mean_power_w", ".0f", False, "power_w"),
    ("CoT", "mean_cost_of_transport", ".2f", False, "cost_of_transport"),
    ("action rate", "mean_action_rate", ".2f", False, "action_rate"),
    ("tilt max, deg", "worst_max_tilt_deg", ".1f", False, "max_tilt_deg"),
    ("trunk contacts/ep", "mean_trunk_contacts_per_episode", ".2f", False, "trunk_contact_events"),
    ("leg contacts/ep", "mean_leg_contacts_per_episode", ".2f", False, "leg_contact_events"),
)
C_BETTER, C_WORSE, C_NEUTRAL = "#2e9e5b", "#d64545", "#6b7a8c"


def _fmt(v, spec):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "-"
    text = format(v, spec)
    return text[1:] if text.startswith("-") and not any(c in "123456789" for c in text) else text  # no "-0%"


def _select(names: list[str], patterns: str | None) -> list[str]:
    if not patterns:
        return names
    wanted = [p.strip() for p in patterns.split(",") if p.strip()]
    pat = ["*" if w == "all" else "rough_*" if w == "rough" else w for w in wanted]
    return [n for n in names if any(fnmatch.fnmatchcase(n, p) for p in pat)]


def print_comparison(isaac: dict, mujoco: dict, names: list[str]):
    w = max(len(n) for n in names) + 2
    print("\nIsaacLab | MuJoCo per scenario (summary over all episodes)")
    for chunk in (ROWS[:7], ROWS[7:]):
        line = "=" * (w + 22 * len(chunk))
        print("\n" + line)
        print(f"{'scenario':<{w}}" + "".join(f"{label:>22}" for label, *_ in chunk))
        print("-" * len(line))
        for n in names:
            si, sm = isaac[n]["summary"], mujoco[n]["summary"]
            cells = []
            for label, key, spec, *_ in chunk:
                cells.append(f"{_fmt(si.get(key), spec)} | {_fmt(sm.get(key), spec)}".rjust(22))
            print(f"{n:<{w}}" + "".join(cells))
        print(line)

    # aggregate
    print("\nMean over the common scenarios:")
    for label, key, spec, *_ in ROWS:
        a, b = scenario_mean(isaac, names, key), scenario_mean(mujoco, names, key)
        diff = "" if a is None or b is None else f"   MuJoCo - IsaacLab = {format(b - a, '+' + spec.lstrip('+'))}"
        print(f"  {label:<22}{_fmt(a, spec):>10} | {_fmt(b, spec):<10}{diff}")
    gaps = sorted(names, key=lambda n: -abs(isaac[n]["summary"]["success_rate"] - mujoco[n]["summary"]["success_rate"]))
    print("\nLargest success gaps (IsaacLab -> MuJoCo):")
    for n in gaps[:8]:
        a, b = isaac[n]["summary"]["success_rate"], mujoco[n]["summary"]["success_rate"]
        if a == b:
            break
        print(f"  {n:<{w}}{a:>6.0%} -> {b:.0%}")
    print(flush=True)


def scenario_mean(rep: dict, names: list[str], key: str):
    """Mean of a summary value over the scenarios (the numbers of "Mean over the common scenarios")."""
    vals = [rep[n]["summary"].get(key) for n in names]
    vals = [v for v in vals if v is not None]
    return float(np.mean(vals)) if vals else None


def episode_values(rep: dict, names: list[str], key: str) -> np.ndarray:
    vals = [e.get(key) for n in names for e in rep[n]["episodes"]]
    return np.array([float(v) for v in vals if v is not None], dtype=float)


def metrics_figure(plt, isaac: dict, mujoco: dict, names: list[str], titles: tuple[str, str]):
    """One panel per metric of "Mean over the common scenarios": the two means as bars, every episode as a dot
    (pooled over the scenarios), and the difference MuJoCo - IsaacLab in the panel title (green = better,
    red = worse than IsaacLab, grey = no better / worse)."""
    from matplotlib.patches import Patch

    ncol = 5
    nrow = -(-len(ROWS) // ncol)
    fig = plt.figure(figsize=(17, 2.9 * nrow + 1.1), facecolor=R.BG)
    total_h = 2.9 * nrow + 1.1
    gs = fig.add_gridspec(nrow, ncol, hspace=0.75, wspace=0.35, left=0.04, right=0.99,
                          top=1.0 - 1.0 / total_h, bottom=0.35 / total_h)  # fmt: skip
    n_ep_i = sum(len(isaac[n]["episodes"]) for n in names)
    n_ep_m = sum(len(mujoco[n]["episodes"]) for n in names)
    fig.suptitle(
        f"Sim-to-sim: mean over {len(names)} common scenario(s) ({titles[0]}: {n_ep_i} episodes, {titles[1]}: {n_ep_m})",
        color="#1f4e8c", fontsize=13, fontweight="bold", x=0.04, ha="left", y=1.0 - 0.15 / total_h, va="top",
    )  # fmt: skip
    fig.legend(
        handles=[Patch(color=C_ISAAC, label=titles[0]), Patch(color=C_MUJOCO, label=titles[1]),
                 plt.Line2D([], [], marker="o", ls="", color="#333333", alpha=0.5, label="episodes")],
        loc="upper right", ncol=3, frameon=False, bbox_to_anchor=(0.99, 1.0 - 0.1 / total_h),
    )  # fmt: skip
    rng = np.random.default_rng(0)
    for k, (label, key, spec, better, ep_key) in enumerate(ROWS):
        ax = fig.add_subplot(gs[k // ncol, k % ncol])
        a, b = scenario_mean(isaac, names, key), scenario_mean(mujoco, names, key)
        is_rate = spec.endswith("%")
        top = 0.0
        for x, (value, rep, color) in enumerate(((a, isaac, C_ISAAC), (b, mujoco, C_MUJOCO))):
            if value is not None:
                ax.bar(x, value, width=0.6, color=color, alpha=0.85)
                ax.text(x, value, _fmt(value, spec), ha="center", va="bottom", fontsize=9, fontweight="bold")
                top = max(top, value)
            if not is_rate:  # 0/1 per episode: no information beyond the bar
                eps = episode_values(rep, names, ep_key)
                if eps.size:
                    ax.scatter(x + rng.uniform(-0.18, 0.18, eps.size), eps, s=14, color="#333333", alpha=0.45, zorder=3, lw=0)
                    top = max(top, float(eps.max()))
        ax.set_xticks([0, 1])
        ax.set_xticklabels(titles, fontsize=8)
        ax.set_xlim(-0.6, 1.6)
        if is_rate:
            ax.set_ylim(0, 1.15)
            ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0%}"))
        elif top > 0:
            ax.set_ylim(min(0.0, ax.get_ylim()[0]), top * 1.18)
        ax.grid(axis="y", alpha=0.3)
        ax.set_title(label, color="#1f4e8c", fontsize=10, loc="left", pad=16)
        if a is not None and b is not None:
            d = b - a
            sign_spec = "+" + spec
            rel = f" ({d / abs(a):+.0%})" if (not is_rate and abs(a) > 1e-9) else ""
            if better is None or abs(d) < 1e-12:
                color = C_NEUTRAL
            else:
                color = C_BETTER if (d > 0) == better else C_WORSE
            ax.text(0.0, 1.02, f"MuJoCo - IsaacLab = {format(d, sign_spec)}{rel}", transform=ax.transAxes,
                    fontsize=8, color=color, va="bottom")  # fmt: skip
    return fig


def _ratio(rep: dict, n: str):
    s = rep[n]["summary"]
    meta = R.get_meta(rep[n], rep[n]["episodes"])
    cmd = meta.get("primary_command")
    if meta.get("motion") != "walk" or not cmd or s.get("mean_forward_speed_mps") is None:
        return None
    return s["mean_forward_speed_mps"] / cmd


def _episode_ratio(rep: dict, n: str) -> list[float]:
    meta = R.get_meta(rep[n], rep[n]["episodes"])
    cmd = meta.get("primary_command")
    if meta.get("motion") != "walk" or not cmd:
        return []
    return [e["forward_speed_mps"] / cmd for e in rep[n]["episodes"] if e.get("forward_speed_mps") is not None]


def comparison_figure(plt, isaac: dict, mujoco: dict, names: list[str], titles: tuple[str, str]):
    """Per scenario (x axis): grouped vertical bars IsaacLab / MuJoCo for success, speed / command, power and trunk
    contacts, every episode as a dot; below, the capability sweeps (stairs / slopes / blocks) of both."""
    from matplotlib.patches import Patch

    n = len(names)
    sweeps_i, sweeps_m = R.find_sweeps({k: isaac[k] for k in names}), R.find_sweeps({k: mujoco[k] for k in names})
    n_sweeps = min(len(sweeps_i), 4)
    label_bars = None  # set below: values above the bars only while they stay readable
    rotate = n > 2
    longest = max(len(k) for k in names)
    labels_h = (0.058 * longest + 0.25) if rotate else 0.35  # height of the scenario names under the bars, inches
    panel_h, sweep_h, gap_w, gap_h = 2.6, 3.2, 0.7, 0.7
    panels = (
        ("success rate", lambda rep, k: rep[k]["summary"]["success_rate"], None, ".0%", True),
        ("speed / command (walks)", lambda rep, k: _ratio(rep, k), _episode_ratio, ".0%", True),
        ("mean power, W", lambda rep, k: rep[k]["summary"].get("mean_power_w"),
         lambda rep, k: [e["power_w"] for e in rep[k]["episodes"] if e.get("power_w") is not None], ".0f", False),
        ("trunk contacts / episode", lambda rep, k: rep[k]["summary"].get("mean_trunk_contacts_per_episode"),
         lambda rep, k: [e["trunk_contact_events"] for e in rep[k]["episodes"] if e.get("trunk_contact_events") is not None], ".2f", False),
    )  # fmt: skip
    # panels in order left to right, two per row (one per row when two do not fit into max_width)
    panel_w = max(6.0, 0.42 * n + 1.0)
    max_width = 26.0
    ncol = max(1, min(2, int((max_width - 1.0 + gap_w) // (panel_w + gap_w))))  # 2 x 2 (1 column if too wide)
    nrow = -(-len(panels) // ncol)
    width = 1.0 + ncol * panel_w + (ncol - 1) * gap_w
    label_bars = n <= (6 if ncol > 1 else 16)
    row_h = panel_h + labels_h
    top_h, bottom_h = 1.0, 0.45
    bars_h = nrow * row_h + (nrow - 1) * gap_h
    total_h = top_h + bars_h + bottom_h + ((gap_h + sweep_h) if n_sweeps else 0.0)
    fig = plt.figure(figsize=(width, total_h), facecolor=R.BG)
    left, right = 0.75 / width, 1.0 - 0.25 / width
    fig.suptitle(f"Sim-to-sim per scenario: {titles[0]} vs {titles[1]}", color="#1f4e8c", fontsize=13, fontweight="bold",
                 x=left, ha="left", y=1.0 - 0.15 / total_h, va="top")  # fmt: skip
    fig.legend(
        handles=[Patch(color=C_ISAAC, label=titles[0]), Patch(color=C_MUJOCO, label=titles[1]),
                 plt.Line2D([], [], marker="o", ls="", color="#333333", alpha=0.5, label="episodes")],
        loc="upper right", ncol=3, frameon=False, bbox_to_anchor=(right, 1.0 - 0.1 / total_h),
    )  # fmt: skip
    x = np.arange(n)
    bw = 0.38
    rng = np.random.default_rng(0)
    for k_panel, (title, getter, ep_getter, spec, as_pct) in enumerate(panels):
        r, c = divmod(k_panel, ncol)
        y_top = top_h + r * (row_h + gap_h)  # inches from the top of the figure
        x0 = 0.75 + c * (panel_w + gap_w)
        ax = fig.add_axes([x0 / width, 1.0 - (y_top + panel_h) / total_h, (panel_w - 0.25) / width, panel_h / total_h])
        top = 0.0
        for off, rep, color in ((-bw / 2, isaac, C_ISAAC), (bw / 2, mujoco, C_MUJOCO)):
            vals = [getter(rep, k) for k in names]
            ax.bar(x + off, [v if v is not None else 0.0 for v in vals], width=bw, color=color, alpha=0.85)
            for xi, v in zip(x, vals):
                if v is None:
                    continue
                top = max(top, v)
                if label_bars:
                    ax.text(xi + off, v, _fmt(v, spec), ha="center", va="bottom", fontsize=7)
            if ep_getter is not None:
                for xi, k in zip(x, names):
                    eps = np.asarray(ep_getter(rep, k), dtype=float)
                    if eps.size:
                        ax.scatter(xi + off + rng.uniform(-bw / 4, bw / 4, eps.size), eps, s=10, color="#333333", alpha=0.45, lw=0, zorder=3)
                        top = max(top, float(eps.max()))
        if as_pct:
            ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0%}"))
        ax.set_ylim(0, max(top, 1e-6) * 1.18 if not (as_pct and top <= 1.0) else 1.15)
        # at most ~1.6 inch per scenario: a few scenarios stay narrow bars in the middle
        span = max(float(n), (panel_w - 0.5) / 1.6)
        pad = (span - n) / 2.0 + 0.5
        ax.set_xlim(-pad, n - 1 + pad)
        ax.grid(axis="y", alpha=0.3)
        ax.set_title(title, color="#1f4e8c", fontsize=10, loc="left")
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=60 if rotate else 0, ha="right" if rotate else "center",
                           rotation_mode="anchor" if rotate else "default", fontsize=8)  # fmt: skip

    if n_sweeps:
        sweeps_top = top_h + bars_h + gap_h
        sub = fig.add_gridspec(1, 4, wspace=0.25, left=left, right=right,
                               top=1.0 - sweeps_top / total_h, bottom=bottom_h / total_h)  # fmt: skip
    if n_sweeps:
        for col, (title, curves) in enumerate(list(sweeps_i.items())[:4]):
            ax = fig.add_subplot(sub[0, col])
            for j, (label, points) in enumerate(curves.items()):
                face = None if j == 0 else "white"  # second curve (e.g. the inverted pyramid): hollow markers, dotted
                ls_i, ls_m = ("-", "--") if j == 0 else (":", ":")
                ax.plot([p for p, _ in points], [isaac[k]["summary"]["success_rate"] for _, k in points], marker="o", ls=ls_i,
                        color=C_ISAAC, mfc=face, label=f"{titles[0]} {label}")  # fmt: skip
                mpts = sweeps_m.get(title, {}).get(label, [])
                if mpts:
                    ax.plot([p for p, _ in mpts], [mujoco[k]["summary"]["success_rate"] for _, k in mpts], marker="s", ls=ls_m,
                            color=C_MUJOCO, mfc=face, label=f"{titles[1]} {label}")  # fmt: skip
            ax.set_ylim(-0.05, 1.05)
            ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0%}"))
            ax.set_title(title + " (success)", color="#1f4e8c", fontsize=10, loc="left")
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7, loc="lower left")
    return fig


class _Tee(io.TextIOBase):
    """Writes to the terminal and keeps a copy of the text (for compare.txt)."""

    def __init__(self, stream):
        self.stream, self.buffer = stream, io.StringIO()

    def write(self, text):
        self.stream.write(text)
        self.buffer.write(text)
        return len(text)

    def flush(self):
        self.stream.flush()

    def getvalue(self) -> str:
        return self.buffer.getvalue()


def default_save_dir(mujoco_report: str, stamp: str) -> str:
    """<folder of the MuJoCo report (the eval/ folder of the policy run)>/sim-to-sim_compare_<date>."""
    return os.path.join(os.path.dirname(os.path.abspath(mujoco_report)), f"sim-to-sim_compare_{stamp}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Compare an IsaacLab and a MuJoCo evaluation report.")
    ap.add_argument("isaac", help="report of unitree_rl_lab scripts/rsl_rl/evaluate.py")
    ap.add_argument("mujoco", help="report of go2_eval evaluate")
    ap.add_argument("--scenarios", default=None, help="only these scenarios (names / glob patterns)")
    ap.add_argument(
        "--save", nargs="?", const="", default=None,
        help="save the figure and the printed comparison into this folder "
             "(without a folder: <MuJoCo report folder>/sim-to-sim_compare_<date>)",
    )  # fmt: skip
    ap.add_argument("--no_show", action="store_true", help="do not open the figure window")
    args = ap.parse_args(argv)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    show = not args.no_show and R.has_display()
    save = None
    if args.save is not None or not show:
        save = os.path.abspath(os.path.expanduser(args.save)) if args.save else default_save_dir(args.mujoco, stamp)

    tee = _Tee(sys.stdout)
    with contextlib.redirect_stdout(tee):
        names, isaac, mujoco = _report(args, stamp)
    if names is None:
        sys.exit("[ERROR] no common scenario")

    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9, "axes.facecolor": "white", "figure.facecolor": R.BG, "savefig.facecolor": R.BG})
    titles = ("IsaacLab", "MuJoCo")
    fig = comparison_figure(plt, isaac, mujoco, names, titles)
    fig_metrics = metrics_figure(plt, isaac, mujoco, names, titles)
    if save:
        os.makedirs(save, exist_ok=True)
        fig.savefig(os.path.join(save, "sim_to_sim.png"), dpi=110)
        fig_metrics.savefig(os.path.join(save, "metrics.png"), dpi=110)
        with open(os.path.join(save, "compare.txt"), "w", encoding="utf-8") as f:
            f.write(tee.getvalue())
        print(
            f"[INFO] Saved to {save}:\n  sim_to_sim.png  (per scenario + sweeps)\n"
            "  metrics.png     (mean over the scenarios, per metric)\n  compare.txt     (comparison tables)"
        )
    if show:
        plt.show()


def _report(args, stamp: str):
    """Print the whole comparison (captured for compare.txt). Returns (names, isaac, mujoco) or (None, ...)."""
    with open(args.isaac, encoding="utf-8") as f:
        isaac_rep = json.load(f)
    with open(args.mujoco, encoding="utf-8") as f:
        mujoco_rep = json.load(f)
    isaac, mujoco = isaac_rep["scenarios"], mujoco_rep["scenarios"]
    common = [n for n in isaac if n in mujoco]
    names = _select(common, args.scenarios)
    print(f"Sim-to-sim comparison, {stamp}")
    print(f"$ eval_compare {shlex.join(sys.argv[1:] if sys.argv[1:] else [])}")
    print(f"[INFO] IsaacLab report: {os.path.abspath(args.isaac)}")
    print(f"[INFO] MuJoCo report:   {os.path.abspath(args.mujoco)}")
    print(f"[INFO] IsaacLab: {isaac_rep.get('checkpoint')} ({isaac_rep.get('date')}, seeds {isaac_rep.get('seeds', isaac_rep.get('seed'))})")
    print(f"[INFO] MuJoCo:   {mujoco_rep.get('checkpoint')} ({mujoco_rep.get('date')}, seeds {mujoco_rep.get('seeds')})")
    only_i = [n for n in isaac if n not in mujoco]
    only_m = [n for n in mujoco if n not in isaac]
    if only_i or only_m:
        print(f"[INFO] only in IsaacLab: {only_i or '-'}\n[INFO] only in MuJoCo: {only_m or '-'}")
    if not names:
        return None, isaac, mujoco
    if isaac_rep.get("speed_override_mps") != mujoco_rep.get("speed_override_mps"):
        print(f"[WARN] different commanded speeds: {isaac_rep.get('speed_override_mps')} vs {mujoco_rep.get('speed_override_mps')}")
    print_comparison(isaac, mujoco, names)
    return names, isaac, mujoco


if __name__ == "__main__":
    main()