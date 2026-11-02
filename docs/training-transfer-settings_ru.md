# Карта настроек: обучение и перенос Go2

Где менять параметры, от которых зависят обучение в IsaacLab и перенос политики в MuJoCo и на робота. Сначала таблица «что → где», ниже по каждому файлу строки с настройками. Значения в листингах приведены для примера.

## Что где менять

### Обучение (unitree_rl_lab, IsaacLab, unitree_ros)

| Что | Файл | Где в файле |
|---|---|---|
| Набор террейнов и их сложность | `IsaacLab/.../terrains/config/rough.py` | `ROUGH_TERRAINS_CFG` |
| Трение террейна | `unitree_rl_lab/.../go2/velocity_env_cfg.py` | `RobotSceneCfg.terrain.physics_material` |
| Стартовый уровень террейна | `velocity_env_cfg.py` | `max_init_terrain_level` |
| Рандомизация трения робота | `velocity_env_cfg.py` | `EventCfg.physics_material` |
| Рандомизация массы базы | `velocity_env_cfg.py` | `EventCfg.add_base_mass` |
| Толчки | `velocity_env_cfg.py` | `EventCfg.push_robot` |
| Разброс позы и скоростей при сбросе | `velocity_env_cfg.py` | `EventCfg.reset_base`, `reset_robot_joints` |
| Диапазоны команд | `velocity_env_cfg.py` | `CommandsCfg.base_velocity.ranges`, `limit_ranges` |
| Масштаб действий | `velocity_env_cfg.py` | `ActionsCfg.JointPositionAction.scale` |
| Наблюдения: масштабы, шум, история | `velocity_env_cfg.py` | `ObservationsCfg.PolicyCfg` |
| Привилегированные наблюдения | `velocity_env_cfg.py` | `ObservationsCfg.CriticCfg` |
| Веса наград | `velocity_env_cfg.py` | `RewardsCfg` |
| Условия завершения эпизода | `velocity_env_cfg.py` | `TerminationsCfg` |
| Curriculum | `velocity_env_cfg.py`, `mdp/curriculums.py`, `isaaclab_tasks/.../velocity/mdp/curriculums.py` | `CurriculumCfg`, `lin_vel_cmd_levels`, `terrain_levels_vel` |
| Шаг физики, decimation, длина эпизода | `velocity_env_cfg.py` | `RobotEnvCfg.__post_init__` |
| Настройки оценки | `velocity_env_cfg.py`, `go2/eval_scenarios.py` | `RobotEvalEnvCfg`, `EVAL_SCENARIOS` |
| PD-коэффициенты, трение суставов | `unitree_rl_lab/.../assets/robots/unitree.py` | `UNITREE_GO2_CFG.actuators` |
| Кривая момент-скорость | `unitree_rl_lab/.../assets/robots/unitree_actuators.py` | `UnitreeActuatorCfg_Go2HV`, `_Go2HV_Calf` |
| Начальная поза, стоячая поза | `unitree.py` | `UNITREE_GO2_CFG.init_state` |
| Импорт URDF: самоколлизии, солвер | `unitree.py` | `UnitreeUrdfFileCfg` |
| Массы, лимиты суставов, коллизии | `unitree_ros/robots/go2_description/urdf/go2_description.urdf` | `<mass>`, `<limit>`, `<collision>` |
| Гиперпараметры PPO | `unitree_rl_lab/.../locomotion/agents/rsl_rl_ppo_cfg.py` | `BasePPORunnerCfg` |
| Что попадает в `deploy.yaml` | `unitree_rl_lab/.../utils/export_deploy_cfg.py` | `cfg[...]` |
| Солвер PhysX | `IsaacLab/.../sim/simulation_cfg.py` | `PhysxCfg` |

### Деплой и симуляция (go2_policy_runner, go2_eval, unitree_mujoco)

| Что | Файл | Где в файле |
|---|---|---|
| Частота lowcmd, таймауты | `go2_policy_runner/config/go2.yaml` | `control_rate`, `lowstate_timeout`, `cmd_vel_timeout` |
| Жёсткие лимиты целевых положений | `go2.yaml` | `joint_limits` |
| Позы и PD режима stand | `go2.yaml` | `poses`, `fsm.stand` |
| Порог падения в режиме walk | `go2.yaml` | `fsm.walk.max_tilt` |
| PD и шаг политики в режиме walk | `params/deploy.yaml` запуска (из `export_deploy_cfg.py`) | `stiffness`, `damping`, `step_dt` |
| Обрезка команды скорости | `params/deploy.yaml` запуска | `commands.base_velocity.ranges` |
| Параметры оценки | `ros2_ws/src/go2_eval/go2_eval/evaluate.py`, `scenarios.py` | аргументы CLI, `HEADING_STIFFNESS` |
| Шаг физики MuJoCo | `unitree_mujoco/simulate_python/config.py` | `SIMULATE_DT` |
| Контактная модель | `unitree_mujoco/unitree_robots/go2/go2.xml` | `<option>` |
| Демпфирование, armature, трение суставов | `go2.xml` | `<default class="go2"><joint>` |
| Пределы моментов | `go2.xml` | `<motor ctrlrange>` |
| Трение и модель контакта стопы | `go2.xml` | `<default class="foot">` |
| Коллизии и масса базы | `go2.xml` | `<body name="base_link">` |
| Пересчёт PD из lowcmd, частота lowstate | `simulate_python/unitree_sdk2py_bridge.py` | `LowCmdHandler`, `RecurrentThread` |
| Сцены оценки: height field, глубина боксов | `simulate_python/eval_scene.py` | константы в начале файла |
| Шаг записи, разбиение контактов на части тела | `simulate_python/eval_sim.py` | аргументы CLI, `HEAD_X`, `FOOT_GEOMS` |

---

## 1. unitree_rl_lab + unitree_ros + IsaacLab

### `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/velocity_env_cfg.py`

Сцена:

```python
terrain = TerrainImporterCfg(
    terrain_type="generator",
    terrain_generator=ROUGH_TERRAINS_CFG,
    max_init_terrain_level=5,
    physics_material=sim_utils.RigidBodyMaterialCfg(
        friction_combine_mode="multiply",
        restitution_combine_mode="multiply",
        static_friction=1.0,
        dynamic_friction=1.0,
    ),
)
height_scanner = RayCasterCfg(
    offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
    pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
)
contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
```

События:

```python
physics_material = EventTerm(func=mdp.randomize_rigid_body_material, mode="startup", params={
    "static_friction_range": (0.3, 1.2),
    "dynamic_friction_range": (0.3, 1.2),
    "restitution_range": (0.0, 0.15),
    "num_buckets": 64,
})
add_base_mass = EventTerm(func=mdp.randomize_rigid_body_mass, mode="startup", params={
    "mass_distribution_params": (-1.0, 3.0),
    "operation": "add",
})
base_external_force_torque = EventTerm(..., params={"force_range": (0.0, 0.0), "torque_range": (-0.0, 0.0)})
reset_base = EventTerm(func=mdp.reset_root_state_uniform, mode="reset", params={
    "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
    "velocity_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0), "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0)},
})
reset_robot_joints = EventTerm(func=mdp.reset_joints_by_scale, mode="reset", params={
    "position_range": (1.0, 1.0),
    "velocity_range": (-1.0, 1.0),
})
push_robot = EventTerm(func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(5.0, 10.0),
    params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}})
```

Команды:

```python
base_velocity = mdp.UniformLevelVelocityCommandCfg(
    resampling_time_range=(10.0, 10.0),
    rel_standing_envs=0.1,
    ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(lin_vel_x=(-0.1, 0.1), lin_vel_y=(-0.1, 0.1), ang_vel_z=(-1, 1)),
    limit_ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(lin_vel_x=(-1.0, 1.0), lin_vel_y=(-0.4, 0.4), ang_vel_z=(-1.0, 1.0)),
)
```

Действия:

```python
JointPositionAction = mdp.JointPositionActionCfg(
    asset_name="robot", joint_names=[".*"], scale=0.25, use_default_offset=True, clip={".*": (-100.0, 100.0)}
)
```

Наблюдения:

```python
# PolicyCfg
base_ang_vel      = ObsTerm(func=mdp.base_ang_vel, scale=0.2, clip=(-100, 100), noise=Unoise(n_min=-0.2, n_max=0.2))
projected_gravity = ObsTerm(func=mdp.projected_gravity, clip=(-100, 100), noise=Unoise(n_min=-0.05, n_max=0.05))
velocity_commands = ObsTerm(func=mdp.generated_commands, clip=(-100, 100), params={"command_name": "base_velocity"})
joint_pos_rel     = ObsTerm(func=mdp.joint_pos_rel, clip=(-100, 100), noise=Unoise(n_min=-0.01, n_max=0.01))
joint_vel_rel     = ObsTerm(func=mdp.joint_vel_rel, scale=0.05, clip=(-100, 100), noise=Unoise(n_min=-0.5, n_max=0.5))
last_action       = ObsTerm(func=mdp.last_action, clip=(-100, 100))
self.history_length = 5
self.enable_corruption = True

# CriticCfg
base_lin_vel   = ObsTerm(func=mdp.base_lin_vel, clip=(-100, 100))
joint_effort   = ObsTerm(func=mdp.joint_effort, scale=0.01, clip=(-100, 100))
height_scanner = ObsTerm(func=mdp.height_scan, clip=(-1.0, 5.0))
```

Награды:

```python
track_lin_vel_xy      = RewTerm(func=mdp.track_lin_vel_xy_exp, weight=1.5, params={"std": math.sqrt(0.25)})
track_ang_vel_z       = RewTerm(func=mdp.track_ang_vel_z_exp, weight=0.75, params={"std": math.sqrt(0.25)})
base_linear_velocity  = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
joint_vel             = RewTerm(func=mdp.joint_vel_l2, weight=-0.001)
joint_acc             = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
joint_torques         = RewTerm(func=mdp.joint_torques_l2, weight=-2e-4)
action_rate           = RewTerm(func=mdp.action_rate_l2, weight=-0.1)
dof_pos_limits        = RewTerm(func=mdp.joint_pos_limits, weight=-10.0)
energy                = RewTerm(func=mdp.energy, weight=-2e-5)
flat_orientation_l2   = RewTerm(func=mdp.flat_orientation_l2, weight=-2.5)
joint_pos             = RewTerm(func=mdp.joint_position_penalty, weight=-0.7, params={"stand_still_scale": 5.0, "velocity_threshold": 0.3})
feet_air_time         = RewTerm(func=mdp.feet_air_time, weight=0.1, params={"threshold": 0.5})
air_time_variance     = RewTerm(func=mdp.air_time_variance_penalty, weight=-1.0)
feet_slide            = RewTerm(func=mdp.feet_slide, weight=-0.1)
undesired_contacts    = RewTerm(func=mdp.undesired_contacts, weight=-1, params={"threshold": 1,
    "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["Head_.*", ".*_hip", ".*_thigh", ".*_calf"])})
```

Завершения и curriculum:

```python
time_out        = DoneTerm(func=mdp.time_out, time_out=True)
base_contact    = DoneTerm(func=mdp.illegal_contact, params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0})
bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 0.8})

terrain_levels     = CurrTerm(func=mdp.terrain_levels_vel)
lin_vel_cmd_levels = CurrTerm(mdp.lin_vel_cmd_levels)
```

Симуляция:

```python
scene = RobotSceneCfg(num_envs=4096, env_spacing=2.5)
self.decimation = 4
self.episode_length_s = 20.0
self.sim.dt = 0.005
self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
```

Оценка (`RobotEvalEnvCfg`):

```python
self.scene.num_envs = 1
self.seed = 42
self.curriculum.terrain_levels = None
self.curriculum.lin_vel_cmd_levels = None
self.observations.policy.enable_corruption = False
self.events.physics_material = None
self.events.add_base_mass = None
self.events.push_robot = None
self.events.reset_robot_joints.params["velocity_range"] = (0.0, 0.0)
self.commands.base_velocity.rel_standing_envs = 0.0
```

### `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/eval_scenarios.py`

```python
SPAWN_CLEARANCE = 0.7
HEADING_STIFFNESS = 0.5
MAX_YAW_RATE = 1.0
BACKWARD_SPEED = 0.5
SIDEWAYS_SPEED = 0.3
TURN_RATE = 0.8
SPRINT_BRAKE_TIME_S = 6.0
SPRINT_EPISODE_S = 10.0

_ROUGH_LEVELS = {
    "pyramid_stairs": (1.0,),
    "pyramid_stairs_inv": (1.0,),
    "boxes": (0.25, 0.5, 0.75, 1.0),
    "random_rough": (None,),
    "hf_pyramid_slope": (0.5, 1.0),
    "hf_pyramid_slope_inv": (0.5, 1.0),
}
STAIRS_STEP_HEIGHTS_CM = (5, *range(10, 21))
SLOPE_ANGLES_DEG = tuple(range(10, 46, 5))
```

### `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`

```python
UNITREE_ROS_DIR = "/home/pavel/unitree-go2-rl/unitree_ros"

class UnitreeArticulationCfg(ArticulationCfg):
    soft_joint_pos_limit_factor = 0.9

class UnitreeUrdfFileCfg(sim_utils.UrdfFileCfg):
    fix_base: bool = False
    activate_contact_sensors: bool = True
    replace_cylinders_with_capsules = True
    joint_drive = sim_utils.UrdfConverterCfg.JointDriveCfg(
        gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0))
    articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4)
    rigid_props = sim_utils.RigidBodyPropertiesCfg(
        linear_damping=0.0, angular_damping=0.0, max_linear_velocity=1000.0, max_angular_velocity=1000.0,
        max_depenetration_velocity=1.0)

UNITREE_GO2_CFG = UnitreeArticulationCfg(
    spawn=UnitreeUrdfFileCfg(asset_path=f"{UNITREE_ROS_DIR}/robots/go2_description/urdf/go2_description.urdf"),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.4),
        joint_pos={".*R_hip_joint": -0.1, ".*L_hip_joint": 0.1,
                   "F[L,R]_thigh_joint": 0.8, "R[L,R]_thigh_joint": 1.0, ".*_calf_joint": -1.5},
    ),
    actuators={
        "GO2HV": unitree_actuators.UnitreeActuatorCfg_Go2HV(
            joint_names_expr=[".*_hip_joint", ".*_thigh_joint"], stiffness=25.0, damping=0.5, friction=0.01),
        "GO2HV_CALF": unitree_actuators.UnitreeActuatorCfg_Go2HV_Calf(
            joint_names_expr=[".*_calf_joint"], stiffness=25.0, damping=0.5, friction=0.01),
    },
)
```

### `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree_actuators.py`

```python
class UnitreeActuatorCfg(DelayedPDActuatorCfg):
    X1: float = 1e9      # скорость, до которой доступен полный момент, рад/с
    X2: float = 1e9      # скорость холостого хода, рад/с
    Y1: float = MISSING  # пиковый момент, момент и скорость в одну сторону, Н·м
    Y2: float = None     # пиковый момент, момент и скорость в разные стороны, Н·м
    Fs: float = 0.0
    Fd: float = 0.0
    Va: float = 0.01

class UnitreeActuatorCfg_Go2HV(UnitreeActuatorCfg):
    X1 = 13.5
    X2 = 30
    Y1 = 20.2
    Y2 = 23.4

class UnitreeActuatorCfg_Go2HV_Calf(UnitreeActuatorCfg):
    X1 = 13.5 / 1.917
    X2 = 30 / 1.917
    Y1 = 20.2 * 1.917
    Y2 = 23.4 * 1.917
```

### `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/mdp/curriculums.py`

```python
if env.common_step_counter % env.max_episode_length == 0:
    if reward > reward_term.weight * 0.8:
        delta_command = torch.tensor([-0.1, 0.1], device=env.device)
```

### `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/agents/rsl_rl_ppo_cfg.py`

```python
num_steps_per_env = 32
max_iterations = 50000
policy = RslRlPpoActorCriticCfg(
    init_noise_std=0.6,
    actor_obs_normalization=True,
    critic_obs_normalization=True,
    actor_hidden_dims=[512, 256, 128],
    critic_hidden_dims=[512, 256, 128],
    activation="elu",
)
algorithm = RslRlPpoAlgorithmCfg(
    value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.1, entropy_coef=0.003,
    num_learning_epochs=5, num_mini_batches=4, learning_rate=2e-4, schedule="adaptive",
    gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
)
```

### `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/utils/export_deploy_cfg.py`

```python
cfg["step_dt"] = env.cfg.sim.dt * env.cfg.decimation
cfg["stiffness"] = stiffness.tolist()
cfg["damping"] = damping.tolist()
cfg["default_joint_pos"] = asset.data.default_joint_pos[0].detach().cpu().numpy().tolist()
cfg["commands"]["base_velocity"]["ranges"] = ranges        # limit_ranges, если они есть
cfg["actions"][action_name] = term_cfg
cfg["observations"][obs_name] = term_cfg
```

### `unitree_ros/robots/go2_description/urdf/go2_description.urdf`

```xml
<mass value="6.921" />   <!-- base -->
<mass value="0.678" />   <!-- *_hip -->
<mass value="1.152" />   <!-- *_thigh -->
<mass value="0.154" />   <!-- *_calf -->
<mass value="0.04" />    <!-- *_foot -->
<mass value="0.089" />   <!-- *_rotor -->

<limit lower="-1.0472" upper="1.0472"   effort="23.7"  velocity="30.1" />   <!-- *_hip_joint -->
<limit lower="-1.5708" upper="3.4907"   effort="23.7"  velocity="30.1" />   <!-- F*_thigh_joint -->
<limit lower="-0.5236" upper="4.5379"   effort="23.7"  velocity="30.1" />   <!-- R*_thigh_joint -->
<limit lower="-2.7227" upper="-0.83776" effort="45.43" velocity="15.70" />  <!-- *_calf_joint -->

<box size="0.3762 0.0935 0.114" />             <!-- base -->
<cylinder radius="0.05" length="0.09" />      <!-- Head_upper -->
<sphere radius="0.047" />                     <!-- Head_lower -->
<cylinder length="0.04" radius="0.046" />     <!-- *_hip -->
<cylinder length="0.12" radius="0.012" />     <!-- *_calf -->
<sphere radius="0.022" />                     <!-- *_foot -->
```

### `IsaacLab/source/isaaclab/isaaclab/terrains/config/rough.py`

```python
ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0), border_width=20.0, num_rows=10, num_cols=20,
    horizontal_scale=0.1, vertical_scale=0.005, slope_threshold=0.75, use_cache=False,
    sub_terrains={
        "pyramid_stairs": MeshPyramidStairsTerrainCfg(proportion=0.2, step_height_range=(0.05, 0.23), step_width=0.3, platform_width=3.0, border_width=1.0, holes=False),
        "pyramid_stairs_inv": MeshInvertedPyramidStairsTerrainCfg(proportion=0.2, step_height_range=(0.05, 0.23), step_width=0.31, platform_width=3.0, border_width=1.0, holes=False),
        "boxes": MeshRandomGridTerrainCfg(proportion=0.2, grid_width=0.45, grid_height_range=(0.05, 0.2), platform_width=2.0),
        "random_rough": HfRandomUniformTerrainCfg(proportion=0.2, noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25),
        "hf_pyramid_slope": HfPyramidSlopedTerrainCfg(proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25),
        "hf_pyramid_slope_inv": HfInvertedPyramidSlopedTerrainCfg(proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25),
    },
)
```

### `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/mdp/curriculums.py`

```python
move_up = distance > terrain.cfg.terrain_generator.size[0] / 2
move_down = distance < torch.norm(command[env_ids, :2], dim=1) * env.max_episode_length_s * 0.5
```

### `IsaacLab/source/isaaclab/isaaclab/sim/simulation_cfg.py`

```python
solver_type: Literal[0, 1] = 1
enable_ccd: bool = False
enable_stabilization: bool = False
bounce_threshold_velocity: float = 0.5
friction_offset_threshold: float = 0.04
friction_correlation_distance: float = 0.025
```

---

## 2. go2_policy_runner + go2_eval

### `ros2_ws/src/go2_policy_runner/go2_policy_runner/config/go2.yaml`

```yaml
policy_controller:
  ros__parameters:
    control_rate: 500.0
    lowstate_timeout: 0.1
    cmd_vel_timeout: 0.5
    joint_limits:
      lower: [-0.838, -1.5708, -2.7227,  -0.838, -1.5708, -2.7227,  -0.838, -0.5236, -2.7227,  -0.838, -0.5236, -2.7227]
      upper: [ 0.838,  3.4907, -0.83776,  0.838,  3.4907, -0.83776,  0.838,  4.5379, -0.83776,  0.838,  4.5379, -0.83776]
    poses:
      lie:   [0.0, 1.36, -2.65,  0.0, 1.36, -2.65,  0.0, 1.36, -2.65,  0.0, 1.36, -2.65]
      stand: [-0.1, 0.8, -1.5,   0.1, 0.8, -1.5,   -0.1, 1.0, -1.5,   0.1, 1.0, -1.5]
    fsm:
      passive:
        kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
      stand:
        kp: [60.0, 80.0, 80.0, 60.0, 80.0, 80.0, 60.0, 80.0, 80.0, 60.0, 80.0, 80.0]
        kd: [5.0, 4.0, 4.0, 5.0, 4.0, 4.0, 5.0, 4.0, 4.0, 5.0, 4.0, 4.0]
        sequence: [lie, stand]
        durations: [1.0, 1.5]
        from:
          walk:
            sequence: [stand]
            durations: [1.0]
      walk:
        type: rl_policy
        max_tilt: 1.0
```

### `<run>/params/deploy.yaml` (пишет `export_deploy_cfg.py`, читает режим `rl_policy`)

```yaml
step_dt: 0.02
stiffness: [25.0, ...]
damping: [0.5, ...]
default_joint_pos: [...]
commands:
  base_velocity:
    ranges:
      lin_vel_x: [-1.0, 1.0]
      lin_vel_y: [-0.4, 0.4]
      ang_vel_z: [-1.0, 1.0]
```

### `ros2_ws/src/go2_policy_runner/go2_policy_runner/src/policy/observation_manager.cpp`

```cpp
obs[i] = std::clamp(obs[i], r[0].as<float>(), r[1].as<float>());   // velocity_commands по deploy.yaml
```

### `ros2_ws/src/go2_eval/go2_eval/evaluate.py`

```bash
--stand_time 3.0
--network_interface lo
--domain_id 0
--sim_args '--dt 0.005 --hfield_resolution 0.02'
```

### `ros2_ws/src/go2_eval/go2_eval/scenarios.py`

```python
HEADING_STIFFNESS = 0.5
MAX_YAW_RATE = 1.0
```

---

## 3. unitree_mujoco

### `unitree_mujoco/unitree_robots/go2/go2.xml`

```xml
<option cone="elliptic" impratio="100" />

<default class="go2">
  <geom friction="0.4" margin="0.001" condim="1"/>
  <joint axis="0 1 0" damping="0.1" armature="0.01" frictionloss="0.2"/>
  <motor ctrlrange="-23.7 23.7"/>
  <default class="abduction">
    <joint axis="1 0 0" range="-1.0472 1.0472"/>
  </default>
  <default class="front_hip">
    <joint range="-1.5708 3.4907"/>
  </default>
  <default class="back_hip">
    <joint range="-0.5236 4.5379"/>
  </default>
  <default class="knee">
    <joint range="-2.7227 -0.83776"/>
    <motor ctrlrange="-45.43 45.43"/>
  </default>
  <default class="foot">
    <geom size="0.022" pos="-0.002 0 -0.213" priority="1" condim="6" friction="0.4 0.02 0.01"/>
  </default>
</default>

<body name="base_link" pos="0 0 0.445">
  <inertial pos="0.021112 0 -0.005366" mass="6.921" diaginertia="0.107027 0.0980771 0.0244531" />
  <geom size="0.1881 0.04675 0.057" type="box" class="collision" />
  <geom size="0.05 0.045" pos="0.285 0 0.01" type="cylinder" class="collision" />
  <geom size="0.047" pos="0.293 0 -0.06" class="collision" />

<jointactuatorfrc name="FR_hip_torque" joint="FR_hip_joint" noise="0.01" />
```

### `unitree_mujoco/simulate_python/config.py`

```python
DOMAIN_ID = 0
INTERFACE = "wlp4s0"
SIMULATE_DT = 0.005
VIEWER_DT = 0.02
```

### `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py`

```python
self.mj_data.ctrl[i] = (msg.motor_cmd[i].tau
    + msg.motor_cmd[i].kp * (msg.motor_cmd[i].q - self.mj_data.sensordata[i])
    + msg.motor_cmd[i].kd * (msg.motor_cmd[i].dq - self.mj_data.sensordata[i + self.num_motor]))

self.lowStateThread = RecurrentThread(interval=self.dt, target=self.PublishLowState, name="sim_lowstate")
```

### `unitree_mujoco/simulate_python/eval_scene.py`

```python
HFIELD_RESOLUTION = 0.02
HFIELD_BASE = 0.5
COLUMN_DEPTH = 1.0
```

### `unitree_mujoco/simulate_python/eval_sim.py`

```python
HEAD_X = 0.25
FOOT_GEOMS = ("FL", "FR", "RL", "RR")
ap.add_argument("--dt", type=float, default=config.SIMULATE_DT)
ap.add_argument("--sample_dt", type=float, default=0.02)
ap.add_argument("--hfield_resolution", type=float, default=eval_scene.HFIELD_RESOLUTION)
```
