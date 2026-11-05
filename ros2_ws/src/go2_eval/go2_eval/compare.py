# Copyright (c) 2026, unitree-go2-rl contributors.
# SPDX-License-Identifier: Apache-2.0
"""Sim-to-sim comparison: an IsaacLab report (unitree_rl_lab evaluate.py) against a MuJoCo report (go2_eval).

Both reports have the same format, so the scenarios are matched by name and the same summary numbers are
put side by side:

    ros2 run go2_eval eval_compare isaac_report.json mujoco_report.json
    ros2 run go2_eval eval_compare isaac.json mujoco.json --save                  # + save into the default folder
    ros2 run go2_eval eval_compare isaac.json mujoco.json --scenarios "rough_pyramid_stairs_*cm" --save plots/

Output: a table in the terminal (IsaacLab | MuJoCo | difference per metric) and one window with tabs:
  * per scenario: vertical bars IsaacLab / MuJoCo (and every episode as a dot) for success rate,
    speed / command, mean power and trunk contacts (as many panels per tab as fit the window);
  * capability sweeps: success over stairs height, slope angle and random blocks of both;
  * metrics: one panel per metric of "Mean over the common scenarios" (the two means as bars, every episode
    as a dot, the difference MuJoCo - IsaacLab: green = better, red = worse than IsaacLab).
Each tab is sized to the window, so it fits a laptop screen. Each report can still be opened with eval_report.py for
the per-scenario and per-episode windows, which look the same for both simulators.

--save [DIR] writes the tabs as they look on a 1920 x 1080 screen (NN_<tab>.png) and everything printed in the
terminal (compare.txt) into DIR.
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
import re
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


def _legend_handles(plt, titles):
    from matplotlib.patches import Patch

    return [Patch(color=C_ISAAC, label=titles[0]), Patch(color=C_MUJOCO, label=titles[1]),
            plt.Line2D([], [], marker="o", ls="", color="#333333", alpha=0.5, label="episodes")]  # fmt: skip


def _draw_metric(ax, plt, isaac, mujoco, names, titles, row, rng, diff_prefix="MuJoCo - IsaacLab = "):
    """One metric of "Mean over the common scenarios": the two means as bars, every episode as a dot."""
    label, key, spec, better, ep_key = row
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
        ax.text(0.0, 1.02, f"{diff_prefix}{format(d, sign_spec)}{rel}", transform=ax.transAxes,
                fontsize=8, color=color, va="bottom")  # fmt: skip


def _metrics_title(isaac, mujoco, names, titles) -> str:
    n_ep_i = sum(len(isaac[n]["episodes"]) for n in names)
    n_ep_m = sum(len(mujoco[n]["episodes"]) for n in names)
    return f"Sim-to-sim: mean over {len(names)} common scenario(s) ({titles[0]}: {n_ep_i} episodes, {titles[1]}: {n_ep_m})"


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


# (title, scenario value, per-episode values, format, value is a fraction)
PANELS = (
    ("success rate", lambda rep, k: rep[k]["summary"]["success_rate"], None, ".0%", True),
    ("speed / command (walks)", lambda rep, k: _ratio(rep, k), _episode_ratio, ".0%", True),
    ("mean power, W", lambda rep, k: rep[k]["summary"].get("mean_power_w"),
     lambda rep, k: [e["power_w"] for e in rep[k]["episodes"] if e.get("power_w") is not None], ".0f", False),
    ("trunk contacts / episode", lambda rep, k: rep[k]["summary"].get("mean_trunk_contacts_per_episode"),
     lambda rep, k: [e["trunk_contact_events"] for e in rep[k]["episodes"] if e.get("trunk_contact_events") is not None], ".2f", False),
)  # fmt: skip


def _labels_h(names: list[str]) -> float:
    """Height of the scenario names under the bars, inches."""
    return (0.058 * max(len(k) for k in names) + 0.25) if len(names) > 2 else 0.35


def _draw_panel(ax, plt, isaac, mujoco, names, panel, rng, label_bars: bool, panel_w: float):
    """One per-scenario panel: grouped vertical bars IsaacLab / MuJoCo, every episode as a dot."""
    title, getter, ep_getter, spec, as_pct = panel
    n = len(names)
    rotate = n > 2
    x = np.arange(n)
    bw = 0.38
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


def _draw_sweep(ax, plt, isaac, mujoco, sweeps_m, title, curves, titles, legend_fs=7):
    """Success over a capability sweep (stairs height, slope angle, blocks) of both simulators."""
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
    ax.legend(fontsize=legend_fs, loc="lower left")


# ---------------------------------------------------------------------------------------------------------------
# Pages: the content split into pages that fit a window (a laptop screen included). Shown as the tabs of one
# window (show_pages) and saved by --save as NN_<tab>.png.


def screen_pages(new_figure, plt, isaac: dict, mujoco: dict, names: list[str], titles: tuple[str, str],
                 width_in: float, height_in: float) -> list[tuple[str, object]]:  # fmt: skip
    """[(tab title, figure)] of width_in x height_in each, with a constrained layout so they follow the window
    when it is resized. new_figure(figsize) makes an empty figure."""
    n = len(names)
    pages = []
    rng = np.random.default_rng(0)

    # per-scenario panels: as many rows as fit the height, two columns while the scenarios fit
    ncol = 2 if 0.42 * n + 1.0 <= width_in / 2 else 1
    row_need = 1.9 + _labels_h(names)  # smallest readable panel + the names under it
    nrow = max(1, min(-(-len(PANELS) // ncol), int((height_in - 0.6) // row_need)))
    per_page = ncol * nrow
    label_bars = n <= (6 if ncol > 1 else 16)
    for start in range(0, len(PANELS), per_page):
        chunk = PANELS[start : start + per_page]
        fig = new_figure((width_in, height_in))
        fig.suptitle(f"Per scenario: {titles[0]} vs {titles[1]}", color="#1f4e8c", fontsize=12, fontweight="bold")
        axes = fig.subplots(nrow, ncol, squeeze=False)
        for k, panel in enumerate(chunk):
            _draw_panel(axes.flat[k], plt, isaac, mujoco, names, panel, rng, label_bars, width_in / ncol)
        for ax in list(axes.flat)[len(chunk) :]:
            ax.set_visible(False)
        axes.flat[0].legend(handles=_legend_handles(plt, titles), loc="upper right", fontsize=8, ncol=3, framealpha=0.85)
        tab = chunk[0][0] if len(chunk) == 1 else "per scenario" + (f" {start // per_page + 1}" if per_page < len(PANELS) else "")
        pages.append((tab, fig))

    sweeps_i, sweeps_m = R.find_sweeps({k: isaac[k] for k in names}), R.find_sweeps({k: mujoco[k] for k in names})
    sweeps = list(sweeps_i.items())[:4]
    if sweeps:
        fig = new_figure((width_in, height_in))
        fig.suptitle("Capability sweeps: success", color="#1f4e8c", fontsize=12, fontweight="bold")
        scol = min(2, len(sweeps))
        axes = fig.subplots(-(-len(sweeps) // scol), scol, squeeze=False)
        for k, (title, curves) in enumerate(sweeps):
            _draw_sweep(axes.flat[k], plt, isaac, mujoco, sweeps_m, title, curves, titles, legend_fs=8)
        for ax in list(axes.flat)[len(sweeps) :]:
            ax.set_visible(False)
        pages.append(("capability sweeps", fig))

    # metrics: 7 x 2 on a wide window, 5 x 3 otherwise; "MuJoCo - IsaacLab" is written once in the title
    mcol = 7 if width_in / height_in >= 1.7 else 5
    fig = new_figure((width_in, height_in))
    fig.suptitle(_metrics_title(isaac, mujoco, names, titles) + ";  Δ = MuJoCo - IsaacLab",
                 color="#1f4e8c", fontsize=12, fontweight="bold")  # fmt: skip
    axes = fig.subplots(-(-len(ROWS) // mcol), mcol, squeeze=False)
    for k, row in enumerate(ROWS):
        _draw_metric(axes.flat[k], plt, isaac, mujoco, names, titles, row, rng, diff_prefix="Δ ")
    for ax in list(axes.flat)[len(ROWS) :]:
        ax.set_visible(False)
    pages.append(("metrics", fig))  # no legend: the bars are named on the x axis
    return pages


# size of the saved pages: the window of a 1920 x 1080 screen (show_pages), at 100 dpi as on the screen
SAVED_PAGE_IN = ((1920 * 0.94 - 10) / 100.0, (1080 * 0.86 - 90) / 100.0)


def _page_file(k: int, tab: str) -> str:
    return f"{k + 1:02d}_" + re.sub(r"[^a-z0-9]+", "_", tab.lower()).strip("_") + ".png"


def _constrained(Figure, figsize):
    try:
        return Figure(figsize=figsize, facecolor=R.BG, layout="constrained")
    except TypeError:  # matplotlib < 3.5
        return Figure(figsize=figsize, facecolor=R.BG, constrained_layout=True)


def _has_tk() -> bool:
    try:
        import tkinter  # noqa: F401
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: F401
    except Exception:
        return False
    return True


def show_pages(plt, isaac: dict, mujoco: dict, names: list[str], titles: tuple[str, str]):
    """One window sized to the screen, one tab per page (screen_pages). Blocks until the window is closed."""
    import tkinter as tk
    from tkinter import ttk

    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    from matplotlib.figure import Figure

    root = tk.Tk()
    root.title("Sim-to-sim comparison")
    root.configure(background=R.BG)
    screen_w, screen_h = root.winfo_screenwidth(), root.winfo_screenheight()
    win_w, win_h = int(screen_w * 0.94), int(screen_h * 0.86)  # room for the panel / dock of the desktop
    root.geometry(f"{win_w}x{win_h}+{(screen_w - win_w) // 2}+10")
    dpi = 100.0
    fig_w, fig_h = (win_w - 10) / dpi, (win_h - 90) / dpi  # minus the tabs and the toolbar
    pages = screen_pages(lambda size: _constrained(Figure, size), plt, isaac, mujoco, names, titles, fig_w, fig_h)

    notebook = ttk.Notebook(root)
    notebook.pack(fill="both", expand=True)
    for tab, fig in pages:
        fig.set_dpi(dpi)
        frame = tk.Frame(notebook, background=R.BG)
        canvas = FigureCanvasTkAgg(fig, master=frame)
        toolbar = NavigationToolbar2Tk(canvas, frame, pack_toolbar=False)  # save / zoom like a normal window
        toolbar.update()
        toolbar.pack(side="bottom", fill="x")  # packed first: stays visible when the window is small
        canvas.get_tk_widget().pack(side="top", fill="both", expand=True)  # the figure follows the window size
        notebook.add(frame, text=f"  {tab}  ")
    root.protocol("WM_DELETE_WINDOW", root.quit)
    root.mainloop()
    root.destroy()


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
        help="save the tabs (PNG) and the printed comparison into this folder "
             "(without a folder: <MuJoCo report folder>/sim-to-sim_compare_<date>)",
    )  # fmt: skip
    ap.add_argument("--no_show", action="store_true", help="do not open the window")
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

    # One window with tabs (show_pages), each page sized to the window; --save writes the same pages.
    tabbed = show and _has_tk()
    if not show or tabbed:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9, "axes.facecolor": "white", "figure.facecolor": R.BG, "savefig.facecolor": R.BG})
    titles = ("IsaacLab", "MuJoCo")
    if save:
        os.makedirs(save, exist_ok=True)
        # the tabs of the window, as they look on a 1920 x 1080 screen
        pages = screen_pages(lambda size: _constrained(plt.figure, size), plt, isaac, mujoco, names, titles, *SAVED_PAGE_IN)
        files = []
        for k, (tab, page) in enumerate(pages):
            files.append(_page_file(k, tab))
            page.savefig(os.path.join(save, files[-1]), dpi=100)
            plt.close(page)
        with open(os.path.join(save, "compare.txt"), "w", encoding="utf-8") as f:
            f.write(tee.getvalue())
        print(f"[INFO] Saved to {save}:\n" + "".join(f"  {name}\n" for name in files) + "  compare.txt  (comparison tables)")
    if show:
        if tabbed:
            show_pages(plt, isaac, mujoco, names, titles)
        else:  # no Tk: the same pages, one matplotlib window each
            screen_pages(lambda size: _constrained(plt.figure, size), plt, isaac, mujoco, names, titles, 12.4, 6.2)
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
