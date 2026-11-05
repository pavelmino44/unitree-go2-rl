# Документация

[English](./README.md) | **Русский**

## Документы проекта

| Документ | Содержание |
|---|---|
| [Настройка и использование](./setup-and-usage_ru.md) | Окружения Python / Conda, обучение в `unitree_rl_lab`, MuJoCo, оценка политики, запуск на реальном роботе |
| [Настройка ROS 2, CycloneDDS и Unitree](./ros2_unitree_setup_ru.md) | Ubuntu 22.04 + ROS 2 Humble, CycloneDDS, `unitree_sdk2`, `unitree_mujoco`, `ros2_ws`, сетевые интерфейсы, типичные проблемы |
| [Соответствие параметров](./parameter-mapping_ru.md) | Где задаются параметры обучения, MuJoCo и `go2_policy_runner` и как они соотносятся |
| [Карта настроек](./settings-map_ru.md) | Что где менять: все настройки, влияющие на обучение и перенос, по файлам |

## Документация компонентов

| Компонент | Документация |
|---|---|
| go2_policy_runner | [README](https://github.com/pavelmino44/go2_policy_runner) — запуск политик в MuJoCo и на реальном роботе (на английском) |
| go2_eval | [README](../ros2_ws/src/go2_eval/README.md) — оценка в MuJoCo на сценариях `unitree_rl_lab`, сравнение sim-to-sim (на английском) |
| unitree_rl_lab | [README](https://github.com/pavelmino44/unitree_rl_lab/tree/pavel/go2-modifications), [Оценка политики](https://github.com/pavelmino44/unitree_rl_lab/blob/pavel/go2-modifications/doc/Evaluation_ru.md) — сценарии и метрики оценки в IsaacLab |
| unitree_mujoco | [README](https://github.com/pavelmino44/unitree_mujoco/tree/pavel/go2-modifications) |
| unitree_ros2 | [README](https://github.com/pavelmino44/unitree_ros2/tree/pavel/go2-modifications) |
| unitree_sdk2 | [README](https://github.com/pavelmino44/unitree_sdk2/tree/pavel/go2-modifications) |
| IsaacLab | [Официальная документация](https://isaac-sim.github.io/IsaacLab) |
