# Parameter mapping: training, simulation, deployment

**English** | [Русский](./parameter-mapping_ru.md)

Where the robot and policy parameters are defined and how they relate across `unitree_rl_lab`, `unitree_mujoco` and `go2_policy_runner`. Paths are relative to the repository root.

## Files

| Short name | File | Contains |
|---|---|---|
| asset | `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py` | `UNITREE_GO2_CFG`: URDF, default pose, actuators, PD gains |
| actuators | `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree_actuators.py` | `UnitreeActuatorCfg_Go2HV`: torque-speed curve, friction, armature |
| env | `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/velocity_env_cfg.py` | Timing, observations, actions, commands, randomization, terminations |
| urdf | `unitree_ros/robots/go2_description/urdf/go2_description.urdf` | Masses, joint limits, effort and velocity limits used in training |
| deploy | `unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity/<run>/params/deploy.yaml` | Exported by training; the only training output read by the controller (with `exported/policy.onnx`) |
| mjcf | `unitree_mujoco/unitree_robots/go2/go2.xml` | MuJoCo model: masses, joint damping/armature/friction, torque limits |
| bridge | `unitree_mujoco/simulate/src/unitree_sdk2_bridge.h` | How MuJoCo turns `lowcmd` into torques |
| runner | `ros2_ws/src/go2_policy_runner/go2_policy_runner/config/go2.yaml` | Controller rate, FSM poses, safety limits, teleop speed limits |

`go2_policy_runner` has no physical parameters of its own. Policy-related values come from `deploy` at startup.

## Policy interface

Must match exactly. `deploy` is generated from `env` and `asset`, so normally only `runner` needs checking.

| Parameter | Training | `deploy` | `runner` |
|---|---|---|---|
| Policy period | env: `sim.dt × decimation` | `step_dt` | `control_rate × step_dt` must be an integer |
| PD gains | asset: `stiffness`, `damping` | `stiffness`, `damping` | used as is in `rl_policy` modes |
| Default pose | asset: `init_state.joint_pos` | `default_joint_pos` (Isaac order) | `poses.stand` (SDK order, map with `joint_ids_map`) |
| Joint order | asset: `joint_sdk_names` | `joint_ids_map` | used as is |
| Action scale / offset | env: `ActionsCfg` | `actions` | used as is |
| Observations, scales, history | env: `ObservationsCfg.PolicyCfg` | `observations` | used as is |
| Command ranges | env: `CommandsCfg.limit_ranges` | `commands.base_velocity.ranges` | commands are clamped to them; `max_vx`, `max_vy`, `max_wz` of `remote_teleop` / `input_teleop` should not exceed them |
| Fall detection | env: `TerminationsCfg.bad_orientation.limit_angle` | — | `fsm.<mode>.max_tilt` |
| Joint limits | urdf: `<limit lower/upper>` | — | `joint_limits` (mjcf uses the same values) |

## Physical model

Not exported to `deploy`. Differences here are the sim-to-real gap.

| Parameter | Training | MuJoCo (`mjcf`) | Real robot reference |
|---|---|---|---|
| Total mass | urdf: sum of `<mass>` (rotors included) | sum of `<inertial mass>` | ≈ 15 kg |
| Mass randomization | env: `EventCfg.add_base_mass` | — | — |
| Torque / speed limit | actuators: `X1`, `X2`, `Y1`, `Y2`, the same for all joints | `<motor ctrlrange>`: hip/thigh and `class="knee"` | urdf: `<limit effort velocity>` per joint |
| Joint armature | actuators / asset: `armature` | `<joint armature>` | — |
| Joint friction | actuators: `Fs`, `Fd`; asset: `friction` | `<joint frictionloss damping>` | — |
| Actuation delay | asset: `min_delay`, `max_delay` | — | present |
| Ground friction | env: `physics_material`, `RobotSceneCfg.terrain` | `class="foot"` `friction` (`priority="1"`) | — |
| Physics / PD rate | env: `sim.dt` | default `timestep`; PD thread in `bridge` | motor drivers |

## Quick checks

```bash
# policy interface actually exported
cat unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity/<run>/params/deploy.yaml

# command curriculum reached the limit ranges: TensorBoard, Curriculum/lin_vel_cmd_levels
tensorboard --logdir unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity

# what the running controller feeds the policy
ros2 topic echo /policy_controller/policy_step
```
