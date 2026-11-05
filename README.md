# Unitree Go2 RL

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04_LTS-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Isaac Sim 5.1.0](https://img.shields.io/badge/Isaac_Sim-5.1.0-76B900?logo=nvidia&logoColor=white)](https://docs.isaacsim.omniverse.nvidia.com/)
[![Isaac Lab 2.3.0](https://img.shields.io/badge/Isaac_Lab-2.3.0-76B900?logo=nvidia&logoColor=white)](https://isaac-sim.github.io/IsaacLab/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.x-0B5FAE)](https://mujoco.org/)
[![Python](https://img.shields.io/badge/Python-3.10_%7C_3.11-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

Research framework for reinforcement learning, autonomous locomotion, and sim-to-sim evaluation of the Unitree Go2 quadruped robot.

The project integrates the main components of the experimental stack as Git submodules. The repository is intended to provide a reproducible environment for development, training, evaluation, and analysis of locomotion controllers.

This version of the project provides a framework for conducting experiments on observation-space design using Actor-Critic PPO (RSL-RL) as the primary reinforcement learning algorithm. It allows users to build and train locomotion policies, experiment with domain randomization, and validate the resulting controllers through sim-to-sim and sim-to-real transfer.

<p align="center">
  <img src="./docs/training_process.png" width="98%" alt="Reinforcement learning training process">
</p>

> [!WARNING]
> **Sim-to-real safety.** Exercise extreme caution when performing sim-to-real transfer of locomotion controllers. Improperly selected domain-randomization parameters can result in unstable or unsafe robot behavior and may cause **serious damage to the robot or its surroundings**.
>
> **Always perform thorough sim-to-sim validation before attempting to deploy a controller on the physical robot.** Never deploy an unvalidated controller directly to a real robot.

---

## Project Structure

```text
unitree-go2-rl/
├── IsaacLab/                 # Training simulator and RL environment
├── docs/                     # Setup and usage documentation
├── ros2_ws/                  # ROS 2 Humble workspace
│   └── src/
│       ├── go2_policy_runner/    # Policy controller for MuJoCo and the real robot
│       ├── go2_eval/             # Policy evaluation in MuJoCo, sim-to-sim comparison
│       └── unitree -> ../../unitree_ros2/cyclonedds_ws/src/unitree   # Unitree message packages
├── unitree_rl_lab/           # Reinforcement learning framework and training configurations
├── unitree_mujoco/           # MuJoCo simulation and sim-to-sim evaluation
├── unitree_ros/              # ROS integration and urdf source
├── unitree_ros2/             # ROS 2 integration
├── unitree_sdk2/             # C++ SDK
├── unitree_sdk2_python/      # Python SDK
└── unitree_metrics/          # Training metrics analysis and visualization
```

## Components

| Component           | Purpose                                                            |
| ------------------- | ------------------------------------------------------------------ |
| IsaacLab (v2.3.0)   | Training simulator and RL environment                              |
| unitree_rl_lab      | RL training framework and locomotion configurations                |
| unitree_mujoco      | MuJoCo simulation and sim-to-sim evaluation                        |
| go2_policy_runner   | ROS 2 controller running trained policies in MuJoCo and on the Go2 |
| go2_eval            | Policy evaluation in MuJoCo on the IsaacLab scenarios (sim-to-sim) |
| unitree_ros         | ROS integration and urdf source                                    |
| unitree_ros2        | ROS 2 integration and Unitree message definitions                  |
| unitree_sdk2        | C++ SDK for Unitree robots                                         |
| unitree_sdk2_python | Python SDK for Unitree robots                                      |
| unitree_metrics     | Training metrics analysis and visualization                        |

## Repository Organization

The repositories listed above are included as Git submodules. `go2_policy_runner` is located inside the ROS 2 workspace at `ros2_ws/src/go2_policy_runner`. `go2_eval` is part of this repository (`ros2_ws/src/go2_eval`).

The main repository records the exact commit of each submodule used by the project. This makes it possible to reproduce a known project state even when the individual component repositories continue to evolve.

The branches specified in `.gitmodules` are used for convenient development and updates, while the commit recorded by the main repository determines the exact version used in a particular revision.

## Requirements

> [!NOTE]
> **Tested on Ubuntu 22.04 LTS (x86_64)** with ROS 2 Humble, Isaac Sim 5.1.0 and Isaac Lab 2.3.0. Other configurations may work but are not tested.

### System

| Requirement | Version | Notes |
| --- | --- | --- |
| OS | Ubuntu 22.04 LTS, x86_64 | |
| GPU | NVIDIA RTX with a recent driver | required by Isaac Sim for training and IsaacLab evaluation; see the [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) |
| Conda | Miniconda / Anaconda | environment `env_isaaclab` |
| Git | with submodule support | |

### Training and evaluation in IsaacLab

| Dependency | Version | Notes |
| --- | --- | --- |
| Isaac Sim | 5.1.0 | installed separately, NVIDIA Isaac Sim license |
| Isaac Lab | 2.3.0 | `IsaacLab` submodule |
| Python | 3.11 | conda environment `env_isaaclab` |
| RSL-RL | installed with Isaac Lab | PPO (Actor-Critic) |
| unitree_rl_lab | | `unitree_rl_lab` submodule, installed in editable mode |

### Simulation in MuJoCo

| Dependency | Version | Notes |
| --- | --- | --- |
| Python | 3.10 | virtual environment `~/unitree_env` |
| MuJoCo Python bindings | 3.x | `pip install mujoco` |
| unitree_sdk2_python | | `unitree_sdk2_python` submodule |
| Cyclone DDS | 0.10.x | required by `unitree_sdk2_python` |
| pygame | | optional, gamepad control |

### Deployment and evaluation with ROS 2

| Dependency | Version | Notes |
| --- | --- | --- |
| ROS 2 | Humble | |
| rmw_cyclonedds_cpp | | Unitree robots communicate over Cyclone DDS |
| unitree_go, unitree_api | | message packages from the `unitree_ros2` submodule |
| ONNX Runtime (C/C++, CPU) | 1.22.0 | downloaded by `go2_policy_runner/scripts/fetch_onnxruntime.sh` |
| Eigen 3, yaml-cpp, ncurses | | `go2_policy_runner` |
| NumPy, Matplotlib, Tk | | `go2_eval` |

System packages for the ROS 2 part:

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp libeigen3-dev libyaml-cpp-dev libncurses-dev \
                 python3-numpy python3-matplotlib python3-tk
```

Step-by-step installation and usage: [docs/setup-and-usage_en.md](docs/setup-and-usage_en.md).

## Usage

### Cloning

Clone the complete project together with all submodules:

```bash
git clone --recurse-submodules git@github.com:pavelmino44/unitree-go2-rl.git
```

If the repository has already been cloned without submodules:

```bash
git submodule update --init --recursive
```

## Documentation

Setup, usage, and component documentation: [docs/README.md](docs/README.md).

## What's New

The project now has reproducible policy tests and a quantitative sim-to-sim transferability assessment:

* **[go2_policy_runner](https://github.com/pavelmino44/go2_policy_runner)** — a ROS 2 controller that runs the trained ONNX policy through the same pipeline in MuJoCo and on the real Go2, with passive / stand / walk modes and command limits taken from the training configuration.
* **[Evaluation in unitree_rl_lab](https://github.com/pavelmino44/unitree_rl_lab/blob/pavel/go2-modifications/doc/Evaluation_en.md)** — a fixed set of test scenarios (one obstacle per scenario: flat, slopes, stairs, boxes, rough terrain), per-episode metrics, reports and a GUI for evaluating policies in IsaacLab.
* **IsaacLab terrains in MuJoCo** — the evaluation terrains are exported from `unitree_rl_lab` and rebuilt as MuJoCo scenes in `unitree_mujoco`, so both simulators test the policy on the same geometry.
* **[go2_eval](ros2_ws/src/go2_eval/README.md)** — runs the same scenarios in MuJoCo through `go2_policy_runner`, computes the same metrics, and compares IsaacLab and MuJoCo results side by side (`eval_compare`) to measure sim-to-sim transferability.

## Development Status

The project is under active development.

The current stack is focused on:

* reinforcement learning for quadruped locomotion;
* Unitree Go2 velocity control;
* training in IsaacLab;
* sim-to-sim evaluation in MuJoCo;
* ROS 2 policy deployment with identical simulation and real-robot pipelines;
* ROS 2 and Unitree SDK integration;
* analysis and visualization of training metrics.

Work in progress:

* LiDAR simulation;
* training of exteroceptive locomotion policies.

Installation and experiment-specific instructions are being documented as the project develops.

<p align="center">
 <img src="./docs/test_stairs.png" width="98%" alt="Unitree Go2 stair climbing test">
</p>

---

## License

The files of this repository (documentation, top-level configuration and the `go2_eval` package) are licensed under the [Apache License 2.0](LICENSE). The software is provided "as is", without warranty of any kind.

The submodules are separate works. Each keeps its own license and copyright notices, and the LICENSE of this repository does not apply to them:

| Component | License |
|---|---|
| IsaacLab | BSD-3-Clause (`isaaclab_mimic`: Apache-2.0) |
| unitree_rl_lab | Apache-2.0 |
| unitree_mujoco | BSD-3-Clause |
| unitree_ros | BSD-3-Clause |
| unitree_ros2 | BSD-3-Clause |
| unitree_sdk2 | BSD-3-Clause |
| unitree_sdk2_python | BSD-3-Clause |
| go2_policy_runner | Apache-2.0 |
| unitree_metrics | MIT |
| go2_eval (this repository) | Apache-2.0 |

`go2_eval` contains code derived from unitree_rl_lab. Each derived file keeps a header that lists the upstream files and the changes made. See [NOTICE](NOTICE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for copyright holders, derived files and software that is installed separately.

This project is not affiliated with, endorsed by, or sponsored by Unitree Robotics or NVIDIA. "Unitree", "Go2", "Isaac Sim" and "Isaac Lab" are used only to describe compatibility.