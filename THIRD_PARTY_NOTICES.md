# Third-party notices

unitree-go2-rl consists of the files of this repository and of Git submodules.

- **This repository:** documentation, top-level configuration and the `go2_eval` package. Licensed under the Apache License 2.0, see [LICENSE](LICENSE).
- **Submodules:** separate works with their own licenses. The LICENSE of this repository does not apply to them. Changes made in the project's forks of the submodules are distributed under the license of the respective submodule.

## Submodules

| Path | Upstream | Fork used by the project | License | License files in the submodule |
|---|---|---|---|---|
| `IsaacLab` | [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) | [pavelmino44/IsaacLab](https://github.com/pavelmino44/IsaacLab/tree/pavel/go2-modifications) | BSD 3-Clause; `isaaclab_mimic`: Apache 2.0 | `LICENSE`, `LICENSE-mimic`, `docs/licenses/` |
| `unitree_rl_lab` | [unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) | [pavelmino44/unitree_rl_lab](https://github.com/pavelmino44/unitree_rl_lab/tree/pavel/go2-modifications) | Apache 2.0 | `LICENCE`, `doc/licenses/` |
| `unitree_mujoco` | [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) | [pavelmino44/unitree_mujoco](https://github.com/pavelmino44/unitree_mujoco/tree/pavel/go2-modifications) | BSD 3-Clause | `LICENSE` |
| `unitree_ros` | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) | [pavelmino44/unitree_ros](https://github.com/pavelmino44/unitree_ros/tree/pavel/go2-modifications) | BSD 3-Clause | `LICENSE` |
| `unitree_ros2` | [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | [pavelmino44/unitree_ros2](https://github.com/pavelmino44/unitree_ros2/tree/pavel/go2-modifications) | BSD 3-Clause | `LICENSE` |
| `unitree_sdk2` | [unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | [pavelmino44/unitree_sdk2](https://github.com/pavelmino44/unitree_sdk2/tree/pavel/go2-modifications) | BSD 3-Clause | `LICENSE`, `licenses/` (CycloneDDS, iceoryx, RapidJSON) |
| `unitree_sdk2_python` | [unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | [pavelmino44/unitree_sdk2_python](https://github.com/pavelmino44/unitree_sdk2_python/tree/pavel/go2-modifications) | BSD 3-Clause | `LICENSE` |
| `ros2_ws/src/go2_policy_runner` | — | [pavelmino44/go2_policy_runner](https://github.com/pavelmino44/go2_policy_runner) | Apache 2.0 | `LICENSE`, `NOTICE`, `THIRD_PARTY_NOTICES.md` |
| `unitree_metrics` | — | [pavelmino44/unitree_metrics](https://github.com/pavelmino44/unitree_metrics) | MIT | `LICENSE` |

`ros2_ws/src/unitree` is a symbolic link to `unitree_ros2/cyclonedds_ws/src/unitree` (the Unitree message packages, BSD 3-Clause).

Copyright holders:

| Component | Copyright |
|---|---|
| IsaacLab | Copyright (c) 2022-2025, The Isaac Lab Project Developers |
| unitree_rl_lab | Copyright (c) 2025, Unitree Robotics Co., Ltd.; Copyright 2024 The Isaac Lab Project Developers |
| unitree_mujoco, unitree_ros, unitree_ros2, unitree_sdk2, unitree_sdk2_python | Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") |
| go2_policy_runner | Copyright (c) 2026, unitree-go2-rl contributors; contains code derived from unitree_rl_lab, unitree_sdk2 and unitree_ros2 (see its `THIRD_PARTY_NOTICES.md`) |
| unitree_metrics | Copyright (c) 2026 Pavel Mino |

## Code in this repository derived from other projects

### unitree_rl_lab — Apache License 2.0

The source is the project's fork, branch `pavel/go2-modifications`: https://github.com/pavelmino44/unitree_rl_lab

Every file listed below has a header that names the files it was derived from and the changes made.

| This repository | Derived from |
|---|---|
| `ros2_ws/src/go2_eval/go2_eval/metrics.py` | `scripts/rsl_rl/evaluate.py` (pure-python helpers, copied unchanged) |
| `ros2_ws/src/go2_eval/go2_eval/eval_report.py` | `scripts/rsl_rl/eval_report.py` (copied unchanged) |
| `ros2_ws/src/go2_eval/go2_eval/evaluate.py` | `scripts/rsl_rl/evaluate.py` |
| `ros2_ws/src/go2_eval/go2_eval/scenarios.py` | `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/eval_scenarios.py` |
| `ros2_ws/src/go2_eval/go2_eval/eval_gui.py` | `scripts/rsl_rl/eval_gui.py` |

The Apache License 2.0 text is in [LICENSE](LICENSE).

## Files added by the project to the submodules

These files live in the submodules and are licensed under the license of the submodule:

| File | License | Note |
|---|---|---|
| `unitree_rl_lab/scripts/rsl_rl/export_mujoco_scenarios.py` | Apache 2.0 | — |
| `unitree_mujoco/simulate_python/eval_scene.py` | BSD 3-Clause | — |
| `unitree_mujoco/simulate_python/eval_sim.py` | BSD 3-Clause | Derived from `simulate_python/unitree_mujoco.py`; keeps the Unitree Robotics copyright notice |

## Software that is not part of the project

The following components are installed separately. They are not included in this repository or its submodules and are subject to their own licenses:

| Component | License |
|---|---|
| NVIDIA Isaac Sim (required by IsaacLab) | NVIDIA Isaac Sim license |
| MuJoCo | Apache 2.0 |
| ROS 2 Humble | Apache 2.0 and others (per package) |
| Eclipse Cyclone DDS | EPL 2.0 / EDL 1.0 |
| ONNX Runtime (downloaded by `go2_policy_runner/scripts/fetch_onnxruntime.sh`) | MIT |
| NumPy, Matplotlib, Tkinter, pygame | BSD / Matplotlib license / PSF / LGPL |

## Trademarks

Unitree, Go2 and related names are trademarks of Unitree Robotics. NVIDIA, Isaac Sim and Isaac Lab are trademarks of NVIDIA Corporation. They are used only to describe compatibility. This project is not affiliated with, endorsed by, or sponsored by Unitree Robotics or NVIDIA.
