# Environment setup: ROS 2 Humble, CycloneDDS, Unitree

**English** | [Русский](./ros2_unitree_setup_ru.md)

Setup of Ubuntu 22.04 + ROS 2 Humble for working with the Unitree Go2 and `unitree_mujoco`.
Paths assume the repository is cloned to `~/unitree-go2-rl`.

Running the policy controller: [`go2_policy_runner`](https://github.com/pavelmino44/go2_policy_runner).

## Contents

1. [Overview](#1-overview)
2. [Clone](#2-clone)
3. [System packages](#3-system-packages)
4. [unitree_sdk2](#4-unitree_sdk2)
5. [unitree_mujoco](#5-unitree_mujoco)
6. [ros2_ws](#6-ros2_ws)
7. [Shell environment](#7-shell-environment)
8. [Network interface selection](#8-network-interface-selection)
9. [Real robot connection](#9-real-robot-connection)
10. [Troubleshooting](#10-troubleshooting)

## 1. Overview

- The Go2 and `unitree_sdk2` use CycloneDDS 0.10.x. ROS 2 with `rmw_cyclonedds_cpp` uses the
  same middleware, so robot topics are available in ROS 2 directly (`rt/lowstate` → `/lowstate`).
- Message definitions (`unitree_go`, `unitree_api`) come from `unitree_ros2` and are built in
  `ros2_ws`.
- CycloneDDS is **not** built from source. The build instructions in the `unitree_ros2` README
  apply to Foxy only; Humble ships a compatible version in `ros-humble-rmw-cyclonedds-cpp`.
- `unitree_mujoco` uses the CycloneDDS bundled with `unitree_sdk2`. It talks to ROS nodes over
  the network, so the two copies coexist as long as they are not loaded into the same process.

| | Real robot | Simulator |
|---|---|---|
| DDS domain | `0` | `0` |
| Network interface | Ethernet to the robot, e.g. `enp3s0` | `lo` |

The interface differs between simulator and robot, so it is selected per session and is not set
globally.

## 2. Clone

```bash
git clone --recurse-submodules https://github.com/pavelmino44/unitree-go2-rl.git ~/unitree-go2-rl
```

`ros2_ws/src/unitree` is a committed symlink to `unitree_ros2/cyclonedds_ws/src/unitree` and
works right after cloning.

## 3. System packages

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rmw-cyclonedds-cpp \
                 libyaml-cpp-dev libeigen3-dev libspdlog-dev libfmt-dev \
                 libboost-all-dev libglfw3-dev
```

## 4. unitree_sdk2

Required by `unitree_mujoco`.

```bash
cd ~/unitree-go2-rl/unitree_sdk2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics -DBUILD_EXAMPLES=OFF
sudo make install
```

## 5. unitree_mujoco

### 5.1. MuJoCo

Download a Linux [MuJoCo release](https://github.com/google-deepmind/mujoco/releases)
(tested with 3.7.0) and link it into `simulate/`:

```bash
mkdir -p ~/.mujoco
tar -xzf mujoco-<version>-linux-x86_64.tar.gz -C ~/.mujoco
ln -s ~/.mujoco/mujoco-<version> ~/unitree-go2-rl/unitree_mujoco/simulate/mujoco
```

### 5.2. Build

Plain CMake project, not a colcon package:

```bash
cd ~/unitree-go2-rl/unitree_mujoco/simulate
mkdir -p build && cd build
cmake .. && make -j$(nproc)
```

The fork (`pavel/go2-modifications`) links the executable with
`-Wl,--disable-new-dtags,-rpath,/opt/unitree_robotics/lib`. This makes the simulator load
CycloneDDS from `unitree_sdk2` even when ROS 2 is sourced. Verify:

```bash
ldd ./unitree_mujoco | grep -E "ddsc|iceoryx"
# expected: libddsc.so.0 and libddscxx.so.0 from /opt/unitree_robotics/lib, no iceoryx
```

### 5.3. Configuration

`simulate/config.yaml`:

```yaml
robot: "go2"
domain_id: 0
interface: "lo"
use_joystick: 1      # gamepad emulates the Unitree remote; 0 if no gamepad
```

### 5.4. Run

```bash
cd ~/unitree-go2-rl/unitree_mujoco/simulate/build
./unitree_mujoco
```

## 6. ros2_ws

Build in a shell without an active conda environment.

```bash
# ONNX Runtime for go2_policy_runner (x86_64 / aarch64 selected automatically)
~/unitree-go2-rl/ros2_ws/src/go2_policy_runner/go2_policy_runner/scripts/fetch_onnxruntime.sh

cd ~/unitree-go2-rl/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Packages built: `unitree_api`, `unitree_go`, `unitree_hg`, `go2_policy_runner_interfaces`,
`go2_policy_runner`.

## 7. Shell environment

Append to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
[ -f ~/unitree-go2-rl/ros2_ws/install/setup.bash ] && source ~/unitree-go2-rl/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

If conda is installed, keep it inactive in ROS shells (`conda deactivate`, or
`conda config --set auto_activate_base false`).

Do **not** add to `~/.bashrc`:

| Entry | Reason |
|---|---|
| `export CYCLONEDDS_URI=...` | Binds every process, including `unitree_mujoco`, to one interface |
| `source ~/unitree_ros2/setup.sh` | Sets `CYCLONEDDS_URI` and may overlay a self-built CycloneDDS |
| `export LD_LIBRARY_PATH=...` | Mixes CycloneDDS versions |

## 8. Network interface selection

| Process | Interface set by |
|---|---|
| `unitree_mujoco` | `simulate/config.yaml` → `interface` |
| `policy_controller`, `remote_teleop` | launch argument `network_interface:=<iface>` |
| `ros2 topic`, `ros2 bag`, rviz, other nodes | `source .../dds_env.sh <iface>` in that shell |

```bash
source ~/unitree-go2-rl/ros2_ws/install/go2_policy_runner/share/go2_policy_runner/scripts/dds_env.sh lo
ros2 topic list            # /lowcmd /lowstate /sportmodestate /wirelesscontroller
ros2 topic echo /lowstate --once
```

`dds_env.sh <iface> [domain_id]` sets `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` and
`CYCLONEDDS_URI` for the current shell only.

## 9. Real robot connection

1. Connect the robot via Ethernet; find the interface name with `ip -br link`.
2. Assign a static address `192.168.123.99/24` to that interface (no gateway).
3. `ping 192.168.123.161`
4. `source .../dds_env.sh <iface>` and `ros2 topic list`. Robot topics, including `/lowstate` and
   `/api/motion_switcher/request`, must be listed.

## 10. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `colcon ERROR ... Duplicate package names: unitree_api` | Extra symlink `unitree_ros2/cyclonedds_ws/src/unitree/unitree` (second `ln -s`) | Remove it |
| CycloneDDS build errors (`idlc` segfault, `libiceoryx_posh.so`) | Building CycloneDDS per the Foxy instructions | Do not build it; remove `src/cyclonedds`, `src/rmw_cyclonedds`, `build`, `install`, `log` in `cyclonedds_ws` |
| `glfw_adapter.h: No such file`, `cannot find -lmujoco` | Missing `simulate/mujoco` link, or built with colcon | Section 5.1; build with `cmake`/`make` |
| `dds_writecdr_impl_common: Assertion ... iox_chunk ... failed` | `unitree_mujoco` loads `libddsc` from ROS 2 | rpath from section 5.2 |
| `<iface>: does not match an available interface`, `Failed to create domain` | `CYCLONEDDS_URI` points to a missing interface | `unset CYCLONEDDS_URI`; remove it from `~/.bashrc` |
| `ros2 topic list` does not show `/lowstate` | Wrong interface or domain, or RMW is not CycloneDDS | `dds_env.sh`; `domain_id: 0`; `echo $RMW_IMPLEMENTATION` |
| `Unknown package 'unitree_go'` | `ros2_ws` not built or not sourced | Section 6; `source ros2_ws/install/setup.bash` |
| `ModuleNotFoundError` from `ros2` / `colcon` | Active conda environment | `conda deactivate` |
| `unitree_sdk2` not found by the simulator CMake | SDK installed to a different prefix | `cmake .. -DCMAKE_PREFIX_PATH=<prefix>` |