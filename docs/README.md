# Documentation

**English** | [Русский](./README_ru.md)

## Project documents

| Document | Content |
|---|---|
| [Setup and usage](./setup-and-usage_en.md) | Python / Conda environments, training in `unitree_rl_lab`, MuJoCo, policy evaluation, running on the real robot |
| [ROS 2, CycloneDDS and Unitree setup](./ros2_unitree_setup_en.md) | Ubuntu 22.04 + ROS 2 Humble, CycloneDDS, `unitree_sdk2`, `unitree_mujoco`, `ros2_ws`, network interfaces, troubleshooting |
| [Parameter mapping](./parameter-mapping_en.md) | Where training, MuJoCo and `go2_policy_runner` parameters are defined and how they correspond |
| [Settings map](./settings-map_en.md) | What to change where: every setting that affects training and transfer, by file |

## Component documentation

| Component | Documentation |
|---|---|
| go2_policy_runner | [README](https://github.com/pavelmino44/go2_policy_runner) — running policies in MuJoCo and on the real robot |
| go2_eval | [README](../ros2_ws/src/go2_eval/README.md) — evaluation in MuJoCo on the `unitree_rl_lab` scenarios, sim-to-sim comparison |
| unitree_rl_lab | [README](https://github.com/pavelmino44/unitree_rl_lab/tree/pavel/go2-modifications), [Evaluation](https://github.com/pavelmino44/unitree_rl_lab/blob/pavel/go2-modifications/doc/Evaluation_en.md) — evaluation scenarios and metrics in IsaacLab |
| unitree_mujoco | [README](https://github.com/pavelmino44/unitree_mujoco/tree/pavel/go2-modifications) |
| unitree_ros2 | [README](https://github.com/pavelmino44/unitree_ros2/tree/pavel/go2-modifications) |
| unitree_sdk2 | [README](https://github.com/pavelmino44/unitree_sdk2/tree/pavel/go2-modifications) |
| IsaacLab | [Official documentation](https://isaac-sim.github.io/IsaacLab) |
