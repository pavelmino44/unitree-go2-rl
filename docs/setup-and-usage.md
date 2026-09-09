# Setup and Usage

This document contains the environment configuration and basic commands required to run the Unitree Go2 reinforcement learning stack.

## Python Environment

The project uses a Python virtual environment for Unitree and MuJoCo-related tools.

The virtual environment is located at:

```text
~/unitree_env
```

Activate it with:

```bash
source ~/unitree_env/bin/activate
```

## IsaacLab and Unitree RL Lab

The reinforcement learning environment is managed using Conda.

Activate the environment with:

```bash
conda activate env_isaaclab
```

This environment is used for training and evaluating the RL policy in IsaacLab and for running the corresponding deployment tools.

---

## MuJoCo

### Start the MuJoCo Simulation

Activate the Python environment and start the MuJoCo simulation:

```bash
source ~/unitree_env/bin/activate

cd ~/unitree-go2-rl/unitree_mujoco/simulate_python/

python3 ./unitree_mujoco.py
```

### Run the `StandGo2Example` ROS 2 Node

The example demonstrates ROS 2 control of the simulated Go2 robot.

```bash
source ~/unitree_env/bin/activate

source ~/unitree-go2-rl/unitree_mujoco/example/ros2/install/setup.bash

cd ~/unitree-go2-rl/unitree_mujoco/example/ros2

./install/stand_go2/bin/stand_go2
```

---

## Real Robot

> **Warning:** Before running low-level control software on the real robot, the Unitree Motion Controller must be disabled. Make sure the robot is in a safe position and cannot fall or move unexpectedly.

### Disable the Motion Controller

The custom `go2_motion_switcher` utility is provided by the modified `unitree_sdk2` repository.

```bash
cd ~/unitree-go2-rl/unitree_sdk2/build/bin

./go2_motion_switcher eno1
```

Use the utility to release/deactivate the Motion Controller before running low-level control applications.

### Run `StandGo2Example` with ROS 2

> **Warning:** The Motion Controller must be disabled before starting this example.

Initialize the ROS 2 environment:

```bash
source ~/unitree-go2-rl/unitree_ros2/setup.sh
```

Then run the example:

```bash
cd ~/unitree-go2-rl/unitree_ros2/example/

./install/unitree_ros2_example/bin/go2_stand_example
```

### Run the Trained Policy on the Real Robot

> **Warning:** The Motion Controller must be disabled before starting the RL controller.

Activate the IsaacLab environment:

```bash
conda activate env_isaaclab
```

Then start the Go2 controller:

```bash
cd ~/unitree-go2-rl/unitree_rl_lab/deploy/robots/go2/build

./go2_ctrl --network eno1
```

The `--network` argument specifies the network interface used to communicate with the robot.

---

## Unitree RL Lab

### Start Training from Scratch

Start a new Go2 velocity-control training run:

```bash
./unitree_rl_lab.sh -t \
    --task Unitree-Go2-Velocity \
    --num_envs 6500 \
    --max_iterations 30000 \
    --headless
```

### Resume Training from a Saved Checkpoint

Resume a previously started training run:

```bash
./unitree_rl_lab.sh -t \
    --task Unitree-Go2-Velocity \
    --resume \
    --load_run 2026-05-15_00-29-16 \
    --num_envs 6500 \
    --max_iterations 20800 \
    --headless
```

The `--load_run` argument specifies the experiment directory from which the training should be resumed.

### Run and Evaluate the Trained Policy

Run the trained policy in IsaacLab:

```bash
./unitree_rl_lab.sh -p \
    --task Unitree-Go2-Velocity \
    --num_envs 30
```

This mode can also be used to export the trained policy to ONNX format for subsequent deployment on the robot.

### Run the Controller in MuJoCo

The trained controller can be tested in the MuJoCo simulation.

Activate the IsaacLab environment:

```bash
conda activate env_isaaclab
```

Then run the controller:

```bash
cd ~/Unitree_Isaac/unitree_rl_lab/deploy/robots/go2/build

./go2_ctrl
```

The controller communicates with the MuJoCo simulation through the configured network and DDS interface.

---

## Network Interfaces

The current configuration uses different network interfaces depending on the target:

| Target                    | Interface |
| ------------------------- | --------- |
| Real Unitree Go2          | `eno1`    |
| MuJoCo / local simulation | `wlp4s0`  |

These values correspond to the current development machine configuration and may need to be changed when reproducing the project on another computer.

## Notes

* The exact software versions and submodule commits are determined by the parent `unitree-go2-rl` repository.
* Training parameters are configured in `unitree_rl_lab`.
* The trained policy can be evaluated in both IsaacLab and MuJoCo.
* Deployment to the real robot requires the corresponding Unitree SDK and ROS 2 environment to be configured.
* Always test low-level control software in simulation before deploying it to the physical robot.
