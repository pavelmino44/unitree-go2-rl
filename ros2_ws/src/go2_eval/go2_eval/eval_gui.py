"""Tk GUI for the MuJoCo evaluation (go2_eval evaluate --gui).

The same window as unitree_rl_lab/scripts/rsl_rl/eval_gui.py (same look, same scenario list, same settings
where they exist), adapted to MuJoCo: the scenarios come from the exported catalog (no Isaac Sim start
needed), the policy is a run folder for go2_policy_runner, the run mode is "headless" or "MuJoCo viewer",
and the DDS interface / domain of the simulator and the controller can be set.

It only collects settings and returns the command-line arguments of evaluate.py, so the GUI path and the
terminal path are the same code downstream.

Saved configurations are JSON files (default folder: ~/.config/go2_eval/configs). The last used settings are
stored in _last.json there and restored on the next start.
"""

from __future__ import annotations

import fnmatch
import glob
import json
import os
import re
import shlex
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter import font as tkfont

CONFIG_DIR = os.path.join(os.path.expanduser("~"), ".config", "go2_eval", "configs")
LAST_CONFIG = os.path.join(CONFIG_DIR, "_last.json")

# --- look (the same as unitree_rl_lab eval_gui.py) ---
BG = "#dbe8f7"
BG_BUTTON = "#c3d7ee"
BG_ACTIVE = "#adc8e6"
BORDER = "#9fb8d4"
FIELD = "#ffffff"
FIELD_READONLY = "#eef4fb"
FG = "#1d2b3a"
FG_HINT = "#5b6f86"
FG_TITLE = "#1f4e8c"
ACCENT = "#2c6db4"
ACCENT_ACTIVE = "#245a96"
SELECT = "#3f7fc4"
FONT_PREFERENCE = ("DejaVu Sans", "Noto Sans", "Liberation Sans", "Ubuntu", "Cantarell", "Helvetica", "Arial", "Lucida Sans")
FONT_SIZE = 10
DIALOG_LOCALE = "en"

MODES = ("headless", "viewer")

DEFAULTS: dict = {
    "catalog": "",
    "scenarios": [],
    "episodes": 5,
    "seeds": "",  # empty = the first exported seed
    "speed": "",
    "policy_dir": "",
    "output": "",
    "mode": "headless",
    "network_interface": "lo",
    "domain_id": "0",
    "extra_args": "",
    "keep_going_after_contact": False,
    "show_report": True,
    "save_plots": False,
    "plots_dir": "",
}


##
# Pure logic
##


def parse_seeds(text: str) -> list[int] | None:
    parts = [p.strip() for p in str(text).split(",") if p.strip()]
    if not parts:
        return []
    try:
        return [int(p) for p in parts]
    except ValueError:
        return None


def parse_episodes(value) -> int | None:
    try:
        n = int(str(value).strip())
    except ValueError:
        return None
    return n if n >= 1 else None


def parse_speed(value) -> float | None:
    try:
        speed = float(str(value).strip())
    except ValueError:
        return None
    return speed if speed > 0.0 else None


def load_catalog(path: str) -> dict | None:
    try:
        with open(os.path.expanduser(path), encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict) and isinstance(data.get("scenarios"), dict):
            return data
    except (OSError, ValueError):
        pass
    return None


def discover_runs(catalog_path: str) -> list[str]:
    """Policy run folders (with exported/policy.onnx) of unitree_rl_lab next to the catalog, newest first."""
    roots = []
    if catalog_path:
        base = os.path.dirname(os.path.dirname(os.path.abspath(os.path.expanduser(catalog_path))))
        roots.append(os.path.join(base, "..", "unitree_rl_lab", "logs", "rsl_rl"))
    roots.append(os.path.join(os.getcwd(), "logs", "rsl_rl"))
    runs = []
    for root in roots:
        for p in glob.glob(os.path.join(root, "*", "*")):
            if os.path.isfile(os.path.join(p, "exported", "policy.onnx")):
                runs.append(os.path.normpath(p))
    return sorted(set(runs), key=lambda p: os.path.basename(p), reverse=True)


def network_interfaces() -> list[str]:
    try:
        names = sorted(os.listdir("/sys/class/net"))
    except OSError:
        names = []
    return ["lo"] + [n for n in names if n != "lo"] + ["auto"]


def validate(cfg: dict) -> str | None:
    if not cfg["catalog"].strip() or not os.path.isfile(os.path.expanduser(cfg["catalog"].strip())):
        return "Scenario catalog not found (export it with export_mujoco_scenarios.py)."
    if not cfg["scenarios"]:
        return "No scenario selected."
    if parse_episodes(cfg["episodes"]) is None:
        return "Episodes per scenario must be an integer >= 1."
    if parse_seeds(cfg["seeds"]) is None:
        return "Seeds must be integers separated by commas, e.g. 42,43,44 (or empty)."
    if str(cfg["speed"]).strip() and parse_speed(cfg["speed"]) is None:
        return "Speed must be a positive number in m/s (or empty for the scenario default)."
    if not cfg["policy_dir"].strip():
        return "No policy folder selected."
    if not os.path.isdir(os.path.expanduser(cfg["policy_dir"].strip())):
        return f"Policy folder not found:\n{cfg['policy_dir']}"
    from go2_eval.evaluate import check_policy_dir

    problem = check_policy_dir(cfg["policy_dir"].strip())
    if problem:
        return problem
    try:
        int(str(cfg["domain_id"]).strip())
    except ValueError:
        return "DDS domain must be an integer."
    if cfg["mode"] not in MODES:
        return "No run mode selected."
    try:
        shlex.split(cfg["extra_args"])
    except ValueError as e:
        return f"Cannot parse the extra arguments: {e}"
    return None


def build_argv(cfg: dict, order: list[str]) -> list[str]:
    chosen = set(cfg["scenarios"])
    names = [n for n in order if n in chosen] + [n for n in cfg["scenarios"] if n not in order]
    argv = [
        "--catalog", os.path.expanduser(cfg["catalog"].strip()),
        "--policy_dir", os.path.expanduser(cfg["policy_dir"].strip()),
        "--scenarios", ",".join(names),
        "--episodes", str(parse_episodes(cfg["episodes"]) or cfg["episodes"]),
    ]  # fmt: skip
    seeds = parse_seeds(cfg["seeds"])
    if seeds:
        argv += ["--eval_seed", ",".join(str(s) for s in seeds)]
    if str(cfg["speed"]).strip():
        argv += ["--speed", str(parse_speed(cfg["speed"]) or cfg["speed"]).strip()]
    if cfg["keep_going_after_contact"]:
        argv.append("--no_contact_termination")
    if not cfg["show_report"]:
        argv.append("--no_report")
    if cfg["save_plots"]:
        folder = cfg["plots_dir"].strip()
        argv += ["--report_save", os.path.expanduser(folder)] if folder else ["--report_save"]
    if cfg["output"].strip():
        argv += ["--output", os.path.expanduser(cfg["output"].strip())]
    if cfg["mode"] == "viewer":
        argv.append("--viewer")
    if cfg["network_interface"].strip() and cfg["network_interface"].strip() != "lo":
        argv += ["--network_interface", cfg["network_interface"].strip()]
    if str(cfg["domain_id"]).strip() not in ("", "0"):
        argv += ["--domain_id", str(cfg["domain_id"]).strip()]
    try:
        argv += shlex.split(cfg["extra_args"])
    except ValueError:
        pass
    return argv


def matches_filter(name: str, text: str) -> bool:
    terms = [t for t in re.split(r"[,\s]+", text.strip().lower()) if t]
    if not terms:
        return True
    low = name.lower()
    return any(fnmatch.fnmatchcase(low, t) if any(c in t for c in "*?[") else t in low for t in terms)


def read_config(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        raw = json.load(f)
    cfg = dict(DEFAULTS)
    for key in DEFAULTS:
        if key in raw:
            cfg[key] = raw[key]
    cfg["scenarios"] = [str(s) for s in cfg["scenarios"]]
    for key in ("catalog", "seeds", "speed", "policy_dir", "output", "network_interface", "domain_id", "extra_args", "plots_dir"):
        cfg[key] = str(cfg[key])
    for key in ("keep_going_after_contact", "show_report", "save_plots"):
        cfg[key] = bool(cfg[key])
    if cfg["mode"] not in MODES:
        cfg["mode"] = "headless"
    return cfg


def write_config(path: str, cfg: dict):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"version": 1, **cfg}, f, indent=2, ensure_ascii=False)


##
# Window
##


class EvalGui:
    def __init__(self, root: tk.Tk, catalog_path: str | None):
        self.root = root
        self.result: list[str] | None = None
        self.catalog: dict = {"scenarios": {}}
        self.order: list[str] = []
        self.visible: list[str] = []
        self.selected: set[str] = set()
        self.config_path: str | None = None

        try:
            root.tk.call("::msgcat::mclocale", DIALOG_LOCALE)
        except tk.TclError:
            pass
        self.bold_font = self._setup_look(root)
        root.title("Go2: policy evaluation in MuJoCo")
        root.minsize(900, 560)
        root.protocol("WM_DELETE_WINDOW", self._cancel)

        names = ("catalog", "episodes", "seeds", "speed", "policy_dir", "output", "plots_dir", "mode",
                 "network_interface", "domain_id", "extra_args", "filter")  # fmt: skip
        self.v = {k: tk.StringVar() for k in names}
        self.b = {k: tk.BooleanVar() for k in ("keep_going_after_contact", "show_report", "save_plots")}
        self.desc_var = tk.StringVar()
        self.status_var = tk.StringVar()
        self.summary_var = tk.StringVar()
        self.cmd_var = tk.StringVar()
        self.cfg_name_var = tk.StringVar(value="Configuration: not saved")

        self._build()
        last = None
        if os.path.isfile(LAST_CONFIG):
            try:
                last = read_config(LAST_CONFIG)
            except (OSError, ValueError):
                last = None
        cfg = dict(last or DEFAULTS)
        if catalog_path and (not cfg["catalog"] or not os.path.isfile(os.path.expanduser(cfg["catalog"]))):
            cfg["catalog"] = catalog_path
        self.v["catalog"].set(cfg["catalog"])
        self.reload_catalog(quiet=True)
        self._apply_cfg(cfg, warn_unknown=False)
        for var in [*self.v.values(), *self.b.values()]:
            var.trace_add("write", self._on_change)
        self._on_change()

    @staticmethod
    def _setup_look(root: tk.Tk):
        available = {f.lower(): f for f in tkfont.families(root)}
        wanted = os.environ.get("EVAL_GUI_FONT")
        family = available.get(wanted.lower(), wanted) if wanted else next(
            (available[f.lower()] for f in FONT_PREFERENCE if f.lower() in available), "sans-serif"
        )
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont", "TkCaptionFont", "TkTooltipFont"):
            tkfont.nametofont(name).configure(family=family, size=FONT_SIZE)
        bold = tkfont.nametofont("TkDefaultFont").copy()
        bold.configure(weight="bold", size=FONT_SIZE + 1)
        style = ttk.Style(root)
        style.theme_use("clam")
        root.configure(background=BG)
        style.configure(".", background=BG, foreground=FG, fieldbackground=FIELD, bordercolor=BORDER,
                        lightcolor=BG, darkcolor=BG, troughcolor=BG_BUTTON, focuscolor=BG)  # fmt: skip
        style.configure("TLabelframe", background=BG, bordercolor=BORDER)
        style.configure("TLabelframe.Label", background=BG, foreground=FG_TITLE, font=bold)
        style.configure("TButton", background=BG_BUTTON, foreground=FG, bordercolor=BORDER, padding=(8, 3))
        style.map("TButton", background=[("pressed", BG_ACTIVE), ("active", BG_ACTIVE)], foreground=[("disabled", FG_HINT)])
        style.configure("Accent.TButton", background=ACCENT, foreground="white", bordercolor=ACCENT_ACTIVE, padding=(14, 3), font=bold)
        style.map("Accent.TButton", background=[("pressed", ACCENT_ACTIVE), ("active", ACCENT_ACTIVE)])
        for widget in ("TEntry", "TCombobox", "TSpinbox"):
            style.configure(widget, fieldbackground=FIELD, background=BG_BUTTON, foreground=FG, bordercolor=BORDER, padding=3)
        style.map("TEntry", fieldbackground=[("readonly", FIELD_READONLY)])
        style.map("TCombobox", fieldbackground=[("disabled", FIELD_READONLY), ("readonly", FIELD)],
                  foreground=[("disabled", FG_HINT)], background=[("active", BG_ACTIVE)])  # fmt: skip
        style.map("TSpinbox", background=[("active", BG_ACTIVE)])
        for widget in ("TRadiobutton", "TCheckbutton"):
            style.configure(widget, background=BG)
            style.map(widget, background=[("active", BG)], indicatorcolor=[("selected", ACCENT), ("!selected", FIELD)])
        style.configure("TScrollbar", background=BG_BUTTON, troughcolor=BG, bordercolor=BORDER, arrowcolor=FG)
        style.map("TScrollbar", background=[("active", BG_ACTIVE)])
        style.configure("Hint.TLabel", foreground=FG_HINT)
        style.configure("Summary.TLabel", foreground=FG_TITLE, font=bold)
        root.option_add("*TCombobox*Listbox.background", FIELD)
        root.option_add("*TCombobox*Listbox.foreground", FG)
        root.option_add("*TCombobox*Listbox.selectBackground", SELECT)
        root.option_add("*TCombobox*Listbox.selectForeground", "white")
        return bold

    def _build(self):
        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)
        pad = {"padx": 6, "pady": 3}

        top = ttk.Frame(root)
        top.grid(row=0, column=0, sticky="ew", **pad)
        ttk.Button(top, text="Load configuration...", command=self._load_config).pack(side="left")
        ttk.Button(top, text="Save configuration...", command=self._save_config).pack(side="left", padx=6)
        ttk.Label(top, textvariable=self.cfg_name_var, style="Hint.TLabel").pack(side="left", padx=10)

        body = ttk.Frame(root)
        body.grid(row=1, column=0, sticky="nsew", **pad)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        # --- left: scenarios ---
        left = ttk.LabelFrame(body, text="Scenarios")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(3, weight=1)

        crow = ttk.Frame(left)
        crow.grid(row=0, column=0, sticky="ew", **pad)
        crow.columnconfigure(1, weight=1)
        ttk.Label(crow, text="Catalog:").grid(row=0, column=0)
        ttk.Entry(crow, textvariable=self.v["catalog"]).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(crow, text="Browse...", width=9, command=self._browse_catalog).grid(row=0, column=2)
        ttk.Button(crow, text="Reload", width=7, command=self.reload_catalog).grid(row=0, column=3, padx=(4, 0))

        frow = ttk.Frame(left)
        frow.grid(row=1, column=0, sticky="ew", **pad)
        frow.columnconfigure(1, weight=1)
        ttk.Label(frow, text="Filter:").grid(row=0, column=0)
        ttk.Entry(frow, textvariable=self.v["filter"]).grid(row=0, column=1, sticky="ew", padx=4)
        presets = ttk.Frame(frow)
        presets.grid(row=1, column=0, columnspan=2, sticky="w", pady=(3, 0))
        ttk.Label(presets, text="Quick:").pack(side="left")
        for label, pattern in (("all", ""), ("flat", "flat*"), ("stairs", "rough_pyramid_stairs*"), ("slopes", "*slope*"), ("boxes", "rough_boxes*")):
            ttk.Button(presets, text=label, width=8, command=lambda p=pattern: self.v["filter"].set(p)).pack(side="left", padx=2)

        lbox = ttk.Frame(left)
        lbox.grid(row=3, column=0, sticky="nsew", **pad)
        lbox.columnconfigure(0, weight=1)
        lbox.rowconfigure(0, weight=1)
        self.lb = tk.Listbox(lbox, selectmode="multiple", exportselection=False, activestyle="none", height=18,
                             background=FIELD, foreground=FG, selectbackground=SELECT, selectforeground="white",
                             borderwidth=0, highlightthickness=1, highlightbackground=BORDER, highlightcolor=ACCENT)  # fmt: skip
        self.lb.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(lbox, orient="vertical", command=self.lb.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.lb.configure(yscrollcommand=sb.set)
        self.lb.bind("<<ListboxSelect>>", self._on_list_select)
        self.lb.bind("<Motion>", self._on_motion)

        btns = ttk.Frame(left)
        btns.grid(row=4, column=0, sticky="ew", **pad)
        ttk.Button(btns, text="Select visible", command=self._select_visible).pack(side="left")
        ttk.Button(btns, text="Deselect visible", command=self._deselect_visible).pack(side="left", padx=4)
        ttk.Button(btns, text="Deselect all", command=self._deselect_all).pack(side="left")
        ttk.Label(left, textvariable=self.desc_var, wraplength=420, style="Hint.TLabel").grid(row=5, column=0, sticky="ew", **pad)
        ttk.Label(left, textvariable=self.status_var, style="Hint.TLabel").grid(row=6, column=0, sticky="ew", **pad)

        # --- right: settings ---
        right = ttk.LabelFrame(body, text="Run settings")
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(1, weight=1)
        r = 0

        def label(text, hint=None):
            ttk.Label(right, text=text).grid(row=r, column=0, sticky="w", **pad)
            if hint:
                ttk.Label(right, text=hint, style="Hint.TLabel").grid(row=r + 1, column=1, columnspan=2, sticky="w", padx=6)

        label("Policy folder:", "run folder (exported/ + params/) or logs/rsl_rl/<experiment> (= latest run)")
        self.run_cb = ttk.Combobox(right, textvariable=self.v["policy_dir"])
        self.run_cb.grid(row=r, column=1, sticky="ew", **pad)
        ttk.Button(right, text="Browse...", width=9, command=self._browse_policy).grid(row=r, column=2, **pad)
        r += 2

        label("Episodes per scenario:")
        ttk.Spinbox(right, from_=1, to=1000, textvariable=self.v["episodes"], width=8).grid(row=r, column=1, sticky="w", **pad)
        r += 1

        label("Seeds:", "terrain seeds (must be exported); empty = the first exported one")
        ttk.Entry(right, textvariable=self.v["seeds"]).grid(row=r, column=1, columnspan=2, sticky="ew", **pad)
        r += 2

        label("Speed (m/s):", "walking speed; empty = scenario default; ignored by flat_turn_360 and flat_max_speed")
        ttk.Entry(right, textvariable=self.v["speed"], width=10).grid(row=r, column=1, sticky="w", **pad)
        r += 2

        label("Report file (JSON):", "empty: <policy run>/eval/mujoco_eval_<time>.json")
        ttk.Entry(right, textvariable=self.v["output"]).grid(row=r, column=1, sticky="ew", **pad)
        ttk.Button(right, text="Choose...", width=9, command=self._browse_output).grid(row=r, column=2, **pad)
        r += 2

        label("Plots (PNG):", "empty folder: next to the JSON report, in a folder with the same name")
        plots = ttk.Frame(right)
        plots.grid(row=r, column=1, sticky="ew", **pad)
        plots.columnconfigure(1, weight=1)
        ttk.Checkbutton(plots, text="Save", variable=self.b["save_plots"]).grid(row=0, column=0, padx=(0, 6))
        self.plots_entry = ttk.Entry(plots, textvariable=self.v["plots_dir"])
        self.plots_entry.grid(row=0, column=1, sticky="ew")
        self.plots_btn = ttk.Button(right, text="Folder...", width=9, command=self._browse_plots)
        self.plots_btn.grid(row=r, column=2, **pad)
        r += 2

        label("Run mode:")
        modes = ttk.Frame(right)
        modes.grid(row=r, column=1, columnspan=2, sticky="w", **pad)
        ttk.Radiobutton(modes, text="Headless (no window)", value="headless", variable=self.v["mode"]).pack(anchor="w")
        ttk.Radiobutton(modes, text="MuJoCo viewer (one window per scenario run)", value="viewer", variable=self.v["mode"]).pack(anchor="w")
        r += 1

        label("Options:")
        opts = ttk.Frame(right)
        opts.grid(row=r, column=1, columnspan=2, sticky="w", **pad)
        ttk.Checkbutton(opts, text="Keep going after base contact (do not end the episode)", variable=self.b["keep_going_after_contact"]).pack(anchor="w")
        ttk.Checkbutton(opts, text="Show the report windows when finished", variable=self.b["show_report"]).pack(anchor="w")
        r += 1

        label("DDS interface:", "of the simulator and go2_policy_runner (both on this computer: lo)")
        dds = ttk.Frame(right)
        dds.grid(row=r, column=1, columnspan=2, sticky="ew", **pad)
        ttk.Combobox(dds, textvariable=self.v["network_interface"], values=network_interfaces(), width=12).pack(side="left")
        ttk.Label(dds, text="  domain:").pack(side="left")
        ttk.Spinbox(dds, from_=0, to=232, textvariable=self.v["domain_id"], width=5).pack(side="left")
        r += 2

        label("Extra arguments:", "e.g. --stand_time 4 --sim_args '--dt 0.002' --sim_python ~/unitree_env/bin/python3")
        ttk.Entry(right, textvariable=self.v["extra_args"]).grid(row=r, column=1, columnspan=2, sticky="ew", **pad)

        bottom = ttk.Frame(root)
        bottom.grid(row=2, column=0, sticky="ew", **pad)
        bottom.columnconfigure(0, weight=1)
        ttk.Label(bottom, textvariable=self.summary_var, style="Summary.TLabel").grid(row=0, column=0, sticky="w")
        cmd = ttk.Frame(bottom)
        cmd.grid(row=1, column=0, sticky="ew", pady=3)
        cmd.columnconfigure(1, weight=1)
        ttk.Label(cmd, text="Equivalent command:").grid(row=0, column=0)
        ttk.Entry(cmd, textvariable=self.cmd_var, state="readonly").grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(cmd, text="Copy", command=self._copy_command).grid(row=0, column=2)
        actions = ttk.Frame(bottom)
        actions.grid(row=0, column=1, rowspan=2, sticky="e", padx=(10, 0))
        ttk.Button(actions, text="Run", style="Accent.TButton", command=self._run).pack(side="left")
        ttk.Button(actions, text="Cancel", command=self._cancel).pack(side="left", padx=4)

    # ---- catalog / list ---------------------------------------------------------------------------------

    def reload_catalog(self, quiet: bool = False):
        path = self.v["catalog"].get().strip()
        catalog = load_catalog(path) if path else None
        if catalog is None:
            self.catalog, self.order = {"scenarios": {}}, []
            self.status_var.set("No catalog: export it with unitree_rl_lab/scripts/rsl_rl/export_mujoco_scenarios.py")
            if not quiet and path:
                messagebox.showerror("Catalog", f"Could not read the catalog:\n{path}")
        else:
            self.catalog, self.order = catalog, list(catalog["scenarios"])
            seeds = sorted({int(s) for sc in catalog["scenarios"].values() for s in sc.get("terrains", {})})
            self.status_var.set(f"Scenarios: {len(self.order)}, exported seeds: {seeds} (exported {catalog.get('created') or '?'})")
            removed = sorted(s for s in self.selected if s not in catalog["scenarios"])
            self.selected -= set(removed)
            if removed and not quiet:
                messagebox.showinfo("List updated", "These selected scenarios are not in the catalog and were deselected:\n" + "\n".join(removed))
        self.run_cb.configure(values=discover_runs(path))
        self._apply_filter()
        if hasattr(self, "_last_filter"):
            self._on_change()

    def _apply_filter(self):
        text = self.v["filter"].get()
        self.visible = [n for n in self.order if matches_filter(n, text)]
        self.lb.delete(0, "end")
        for name in self.visible:
            self.lb.insert("end", name)
        self._show_selection()

    def _show_selection(self):
        self.lb.selection_clear(0, "end")
        for i, name in enumerate(self.visible):
            if name in self.selected:
                self.lb.selection_set(i)

    def _on_list_select(self, _event=None):
        current = set(self.lb.curselection())
        for i, name in enumerate(self.visible):
            (self.selected.add if i in current else self.selected.discard)(name)
        self._on_change()

    def _on_motion(self, event):
        i = self.lb.nearest(event.y)
        if 0 <= i < len(self.visible):
            name = self.visible[i]
            sc = self.catalog["scenarios"].get(name, {})
            seeds = sorted(sc.get("terrains", {}), key=int)
            self.desc_var.set(f"{name}: {sc.get('description', '')}  [seeds {', '.join(seeds)}]")

    def _select_visible(self):
        self.selected |= set(self.visible)
        self._show_selection()
        self._on_change()

    def _deselect_visible(self):
        self.selected -= set(self.visible)
        self._show_selection()
        self._on_change()

    def _deselect_all(self):
        self.selected.clear()
        self._show_selection()
        self._on_change()

    # ---- configuration ----------------------------------------------------------------------------------

    def _collect(self) -> dict:
        v = {k: var.get() for k, var in self.v.items()}
        scenarios = [n for n in self.order if n in self.selected] + sorted(n for n in self.selected if n not in self.order)
        cfg = {k: v[k].strip() for k in ("catalog", "episodes", "seeds", "speed", "policy_dir", "output", "plots_dir",
                                         "network_interface", "domain_id", "extra_args")}  # fmt: skip
        cfg.update(scenarios=scenarios, mode=v["mode"] or "headless", **{k: bool(b.get()) for k, b in self.b.items()})
        return cfg

    def _apply_cfg(self, cfg: dict, warn_unknown: bool = True):
        cfg = {**DEFAULTS, **cfg}
        wanted = [str(s) for s in cfg["scenarios"]]
        unknown = [s for s in wanted if self.order and s not in self.catalog["scenarios"]]
        self.selected = {s for s in wanted if s not in unknown}
        for key in ("catalog", "episodes", "seeds", "speed", "policy_dir", "output", "plots_dir", "mode",
                    "network_interface", "domain_id", "extra_args"):  # fmt: skip
            self.v[key].set(str(cfg[key]))
        for key in self.b:
            self.b[key].set(bool(cfg[key]))
        self._apply_filter()
        if unknown and warn_unknown:
            messagebox.showwarning("Configuration", "These scenarios are not in the current catalog and were skipped:\n" + "\n".join(unknown))

    def _save_config(self):
        os.makedirs(CONFIG_DIR, exist_ok=True)
        path = filedialog.asksaveasfilename(title="Save configuration", initialdir=CONFIG_DIR,
                                            initialfile=os.path.basename(self.config_path) if self.config_path else "config.json",
                                            defaultextension=".json", filetypes=[("JSON", "*.json"), ("All files", "*.*")])  # fmt: skip
        if not path:
            return
        try:
            write_config(path, self._collect())
        except OSError as e:
            messagebox.showerror("Configuration", f"Could not save:\n{e}")
            return
        self.config_path = path
        self.cfg_name_var.set(f"Configuration: {os.path.basename(path)}")

    def _load_config(self):
        os.makedirs(CONFIG_DIR, exist_ok=True)
        path = filedialog.askopenfilename(title="Load configuration", initialdir=CONFIG_DIR, filetypes=[("JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            cfg = read_config(path)
        except (OSError, ValueError, TypeError) as e:
            messagebox.showerror("Configuration", f"Could not read the file:\n{e}")
            return
        if cfg["catalog"] and cfg["catalog"] != self.v["catalog"].get():
            self.v["catalog"].set(cfg["catalog"])
            self.reload_catalog(quiet=True)
        self._apply_cfg(cfg)
        self.config_path = path
        self.cfg_name_var.set(f"Configuration: {os.path.basename(path)}")

    # ---- other callbacks ----------------------------------------------------------------------------------

    def _browse_catalog(self):
        path = filedialog.askopenfilename(title="Scenario catalog", filetypes=[("catalog.json", "*.json"), ("All files", "*.*")])
        if path:
            self.v["catalog"].set(path)
            self.reload_catalog()

    def _browse_policy(self):
        start = self.v["policy_dir"].get().strip() or os.getcwd()
        path = filedialog.askdirectory(title="Policy folder", initialdir=os.path.expanduser(start), mustexist=True)
        if path:
            self.v["policy_dir"].set(path)

    def _browse_output(self):
        path = filedialog.asksaveasfilename(title="Report file", defaultextension=".json", filetypes=[("JSON", "*.json")])
        if path:
            self.v["output"].set(path)

    def _browse_plots(self):
        path = filedialog.askdirectory(title="Folder for the plots", mustexist=False)
        if path:
            self.v["plots_dir"].set(path)
            self.b["save_plots"].set(True)

    def _copy_command(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.cmd_var.get())

    def _on_change(self, *_):
        if self.v["filter"].get() != getattr(self, "_last_filter", None):
            self._last_filter = self.v["filter"].get()
            self._apply_filter()
        cfg = self._collect()
        state = ["!disabled"] if cfg["save_plots"] else ["disabled"]
        self.plots_entry.state(state)
        self.plots_btn.state(state)
        n = len(cfg["scenarios"])
        seeds = parse_seeds(cfg["seeds"])
        n_seeds = len(seeds) if seeds else 1
        episodes = parse_episodes(cfg["episodes"]) or 0
        runs = n * n_seeds
        est = 0.0
        for name in cfg["scenarios"]:  # rough duration: the episodes run in real time, plus standing up
            length = self.catalog["scenarios"].get(name, {}).get("episode_length_s") or 20.0
            est += n_seeds * episodes * (0.6 * length + 5.0)
        hint = f" (about {est / 60:.0f} min)" if est > 0 else ""
        self.summary_var.set(f"Scenarios: {n} x seeds: {n_seeds} = runs: {runs}; total episodes: {runs * episodes}{hint}")
        self.cmd_var.set("ros2 run go2_eval evaluate " + shlex.join(build_argv(cfg, self.order)))

    def _run(self):
        cfg = self._collect()
        error = validate(cfg)
        if error:
            messagebox.showerror("Check the settings", error)
            return
        seeds = parse_seeds(cfg["seeds"]) or []
        missing = [(n, s) for n in cfg["scenarios"] for s in seeds
                   if str(s) not in self.catalog["scenarios"].get(n, {}).get("terrains", {})]  # fmt: skip
        if missing:
            messagebox.showerror("Seeds", "No exported terrain for:\n" + "\n".join(f"{n} seed {s}" for n, s in missing[:12])
                                 + "\n\nExport them with export_mujoco_scenarios.py --append --eval_seed ...")  # fmt: skip
            return
        try:
            write_config(LAST_CONFIG, cfg)
        except OSError:
            pass
        self.result = build_argv(cfg, self.order)
        self.root.destroy()

    def _cancel(self):
        self.result = None
        self.root.destroy()


def run_gui(catalog_path: str | None) -> list[str] | None:
    """Open the window. Returns the evaluate.py arguments, or None if it was closed / cancelled."""
    try:
        root = tk.Tk()
    except tk.TclError as e:
        raise SystemExit(f"[ERROR] Could not open a window (no display?): {e}. Run without --gui.") from e
    gui = EvalGui(root, catalog_path)
    root.mainloop()
    return gui.result