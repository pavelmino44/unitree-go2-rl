# Соответствие параметров: обучение, симуляция, запуск

[English](./parameter-mapping_en.md) | **Русский**

Где задаются параметры робота и политики и как они соотносятся между `unitree_rl_lab`, `unitree_mujoco` и `go2_policy_runner`. Пути указаны от корня репозитория.

## Файлы

| Обозначение | Файл | Содержит |
|---|---|---|
| asset | `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py` | `UNITREE_GO2_CFG`: URDF, начальная поза, приводы, коэффициенты PD |
| actuators | `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree_actuators.py` | `UnitreeActuatorCfg_Go2HV`: кривая «момент — скорость», трение, инерция ротора |
| env | `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/velocity_env_cfg.py` | Тайминги, наблюдения, действия, команды, рандомизация, условия завершения |
| urdf | `unitree_ros/robots/go2_description/urdf/go2_description.urdf` | Массы, пределы суставов, моментов и скоростей, используемые при обучении |
| deploy | `unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity/<run>/params/deploy.yaml` | Экспорт обучения; единственный результат обучения, который читает контроллер (вместе с `exported/policy.onnx`) |
| mjcf | `unitree_mujoco/unitree_robots/go2/go2.xml` | Модель MuJoCo: массы, демпфирование, инерция ротора и трение суставов, пределы моментов |
| bridge | `unitree_mujoco/simulate/src/unitree_sdk2_bridge.h` | Как MuJoCo превращает `lowcmd` в моменты |
| runner | `ros2_ws/src/go2_policy_runner/go2_policy_runner/config/go2.yaml` | Частота контроллера, позы FSM, ограничения безопасности, пределы скорости телеуправления |

У `go2_policy_runner` нет собственных физических параметров. Всё, что относится к политике, он читает из `deploy` при запуске.

## Интерфейс политики

Должен совпадать точно. `deploy` генерируется из `env` и `asset`, поэтому обычно проверять нужно только `runner`.

| Параметр | Обучение | `deploy` | `runner` |
|---|---|---|---|
| Период политики | env: `sim.dt × decimation` | `step_dt` | `control_rate × step_dt` должно быть целым |
| Коэффициенты PD | asset: `stiffness`, `damping` | `stiffness`, `damping` | используются как есть в режимах `rl_policy` |
| Начальная поза | asset: `init_state.joint_pos` | `default_joint_pos` (порядок Isaac) | `poses.stand` (порядок SDK, сопоставлять через `joint_ids_map`) |
| Порядок суставов | asset: `joint_sdk_names` | `joint_ids_map` | используется как есть |
| Масштаб и смещение действий | env: `ActionsCfg` | `actions` | используются как есть |
| Наблюдения, масштабы, история | env: `ObservationsCfg.PolicyCfg` | `observations` | используются как есть |
| Диапазоны команд | env: `CommandsCfg.limit_ranges` | `commands.base_velocity.ranges` | команды обрезаются по ним; `max_vx`, `max_vy`, `max_wz` у `remote_teleop` / `input_teleop` не должны их превышать |
| Обнаружение падения | env: `TerminationsCfg.bad_orientation.limit_angle` | — | `fsm.<mode>.max_tilt` |
| Пределы суставов | urdf: `<limit lower/upper>` | — | `joint_limits` (в mjcf те же значения) |

## Физическая модель

В `deploy` не попадает. Расхождения здесь и есть разрыв между симуляцией и реальным роботом.

| Параметр | Обучение | MuJoCo (`mjcf`) | Ориентир для реального робота |
|---|---|---|---|
| Общая масса | urdf: сумма `<mass>` (включая роторы) | сумма `<inertial mass>` | ≈ 15 кг |
| Рандомизация массы | env: `EventCfg.add_base_mass` | — | — |
| Предел момента и скорости | actuators: `X1`, `X2`, `Y1`, `Y2`, одинаковые для всех суставов | `<motor ctrlrange>`: hip/thigh и `class="knee"` | urdf: `<limit effort velocity>` по суставам |
| Инерция ротора | actuators / asset: `armature` | `<joint armature>` | — |
| Трение в суставах | actuators: `Fs`, `Fd`; asset: `friction` | `<joint frictionloss damping>` | — |
| Задержка управления | asset: `min_delay`, `max_delay` | — | есть |
| Трение о поверхность | env: `physics_material`, `RobotSceneCfg.terrain` | `friction` в `class="foot"` (`priority="1"`) | — |
| Частота физики и PD | env: `sim.dt` | `timestep` по умолчанию; поток PD в `bridge` | драйверы моторов |

## Быстрая проверка

```bash
# что фактически экспортировано для политики
cat unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity/<run>/params/deploy.yaml

# дошёл ли куррикулум команд до предельных диапазонов: TensorBoard, Curriculum/lin_vel_cmd_levels
tensorboard --logdir unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity

# что работающий контроллер подаёт в политику
ros2 topic echo /policy_controller/policy_step
```
