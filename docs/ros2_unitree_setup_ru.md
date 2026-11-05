# Настройка окружения: ROS 2 Humble, CycloneDDS, Unitree

[English](./ros2_unitree_setup_en.md) | **Русский**

Настройка Ubuntu 22.04 + ROS 2 Humble для работы с Unitree Go2 и `unitree_mujoco`.
Пути указаны для репозитория, склонированного в `~/unitree-go2-rl`.

Запуск контроллера политики: [`go2_policy_runner`](https://github.com/pavelmino44/go2_policy_runner).

## Содержание

1. [Общая схема](#1-общая-схема)
2. [Клонирование](#2-клонирование)
3. [Системные пакеты](#3-системные-пакеты)
4. [unitree_sdk2](#4-unitree_sdk2)
5. [unitree_mujoco](#5-unitree_mujoco)
6. [ros2_ws](#6-ros2_ws)
7. [Окружение shell](#7-окружение-shell)
8. [Выбор сетевого интерфейса](#8-выбор-сетевого-интерфейса)
9. [Подключение к роботу](#9-подключение-к-роботу)
10. [Типичные проблемы](#10-типичные-проблемы)

## 1. Общая схема

- Go2 и `unitree_sdk2` используют CycloneDDS 0.10.x. ROS 2 с `rmw_cyclonedds_cpp` использует
  тот же DDS, поэтому топики робота доступны в ROS 2 напрямую (`rt/lowstate` → `/lowstate`).
- Описания сообщений (`unitree_go`, `unitree_api`) берутся из `unitree_ros2` и собираются в
  `ros2_ws`.
- CycloneDDS из исходников **не** собирается. Инструкция по сборке в README `unitree_ros2`
  относится только к Foxy; в Humble совместимая версия входит в `ros-humble-rmw-cyclonedds-cpp`.
- `unitree_mujoco` использует CycloneDDS из `unitree_sdk2`. С ROS-узлами он общается по сети,
  поэтому две копии библиотеки не конфликтуют, пока не загружены в один процесс.

| | Реальный робот | Симулятор |
|---|---|---|
| DDS domain | `0` | `0` |
| Сетевой интерфейс | Ethernet к роботу, например `enp3s0` | `lo` |

Интерфейс у симулятора и робота разный, поэтому он выбирается на каждый сеанс и глобально не
задаётся.

## 2. Клонирование

```bash
git clone --recurse-submodules https://github.com/pavelmino44/unitree-go2-rl.git ~/unitree-go2-rl
```

`ros2_ws/src/unitree` — закоммиченный симлинк на `unitree_ros2/cyclonedds_ws/src/unitree`,
работает сразу после клонирования.

## 3. Системные пакеты

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rmw-cyclonedds-cpp \
                 libyaml-cpp-dev libeigen3-dev libspdlog-dev libfmt-dev \
                 libboost-all-dev libglfw3-dev
```

## 4. unitree_sdk2

Нужен для `unitree_mujoco`.

```bash
cd ~/unitree-go2-rl/unitree_sdk2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics -DBUILD_EXAMPLES=OFF
sudo make install
```

## 5. unitree_mujoco

### 5.1. MuJoCo

Скачать [релиз MuJoCo](https://github.com/google-deepmind/mujoco/releases) для Linux
(проверено с 3.7.0) и подключить симлинком в `simulate/`:

```bash
mkdir -p ~/.mujoco
tar -xzf mujoco-<версия>-linux-x86_64.tar.gz -C ~/.mujoco
ln -s ~/.mujoco/mujoco-<версия> ~/unitree-go2-rl/unitree_mujoco/simulate/mujoco
```

### 5.2. Сборка

Обычный CMake-проект, не colcon-пакет:

```bash
cd ~/unitree-go2-rl/unitree_mujoco/simulate
mkdir -p build && cd build
cmake .. && make -j$(nproc)
```

В форке (`pavel/go2-modifications`) исполняемый файл линкуется с
`-Wl,--disable-new-dtags,-rpath,/opt/unitree_robotics/lib`, поэтому симулятор загружает
CycloneDDS из `unitree_sdk2`, даже если подключён ROS 2. Проверка:

```bash
ldd ./unitree_mujoco | grep -E "ddsc|iceoryx"
# ожидается: libddsc.so.0 и libddscxx.so.0 из /opt/unitree_robotics/lib, iceoryx нет
```

### 5.3. Конфигурация

`simulate/config.yaml`:

```yaml
robot: "go2"
domain_id: 0
interface: "lo"
use_joystick: 1      # геймпад эмулирует пульт Unitree; 0, если геймпада нет
```

### 5.4. Запуск

```bash
cd ~/unitree-go2-rl/unitree_mujoco/simulate/build
./unitree_mujoco
```

## 6. ros2_ws

Собирать в терминале без активного conda-окружения.

```bash
# ONNX Runtime для go2_policy_runner (x86_64 / aarch64 выбирается автоматически)
~/unitree-go2-rl/ros2_ws/src/go2_policy_runner/go2_policy_runner/scripts/fetch_onnxruntime.sh

cd ~/unitree-go2-rl/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Собираются пакеты `unitree_api`, `unitree_go`, `unitree_hg`, `go2_policy_runner_interfaces`,
`go2_policy_runner`.

## 7. Окружение shell

Добавить в конец `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
[ -f ~/unitree-go2-rl/ros2_ws/install/setup.bash ] && source ~/unitree-go2-rl/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Если установлена conda, в ROS-терминалах она должна быть неактивна (`conda deactivate` или
`conda config --set auto_activate_base false`).

В `~/.bashrc` **не** добавлять:

| Строка | Причина |
|---|---|
| `export CYCLONEDDS_URI=...` | Привязывает все процессы, включая `unitree_mujoco`, к одному интерфейсу |
| `source ~/unitree_ros2/setup.sh` | Выставляет `CYCLONEDDS_URI` и может подключить самосборный CycloneDDS |
| `export LD_LIBRARY_PATH=...` | Смешивает версии CycloneDDS |

## 8. Выбор сетевого интерфейса

| Процесс | Где задаётся интерфейс |
|---|---|
| `unitree_mujoco` | `simulate/config.yaml` → `interface` |
| `policy_controller`, `remote_teleop` | launch-аргумент `network_interface:=<iface>` |
| `ros2 topic`, `ros2 bag`, rviz, прочие узлы | `source .../dds_env.sh <iface>` в этом терминале |

```bash
source ~/unitree-go2-rl/ros2_ws/install/go2_policy_runner/share/go2_policy_runner/scripts/dds_env.sh lo
ros2 topic list            # /lowcmd /lowstate /sportmodestate /wirelesscontroller
ros2 topic echo /lowstate --once
```

`dds_env.sh <iface> [domain_id]` выставляет `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` и
`CYCLONEDDS_URI` только для текущего терминала.

## 9. Подключение к роботу

1. Подключить робота по Ethernet, узнать имя интерфейса: `ip -br link`.
2. Назначить интерфейсу статический адрес `192.168.123.99/24` (без шлюза).
3. `ping 192.168.123.161`
4. `source .../dds_env.sh <iface>` и `ros2 topic list`. В списке должны быть топики робота,
   в том числе `/lowstate` и `/api/motion_switcher/request`.

## 10. Типичные проблемы

| Симптом | Причина | Решение |
|---|---|---|
| `colcon ERROR ... Duplicate package names: unitree_api` | Лишний симлинк `unitree_ros2/cyclonedds_ws/src/unitree/unitree` (повторный `ln -s`) | Удалить его |
| Ошибки сборки CycloneDDS (`idlc` segfault, `libiceoryx_posh.so`) | Сборка CycloneDDS по инструкции для Foxy | Не собирать; удалить `src/cyclonedds`, `src/rmw_cyclonedds`, `build`, `install`, `log` в `cyclonedds_ws` |
| `glfw_adapter.h: No such file`, `cannot find -lmujoco` | Нет симлинка `simulate/mujoco` или сборка через colcon | Раздел 5.1; собирать через `cmake`/`make` |
| `dds_writecdr_impl_common: Assertion ... iox_chunk ... failed` | `unitree_mujoco` загружает `libddsc` из ROS 2 | rpath из раздела 5.2 |
| `<iface>: does not match an available interface`, `Failed to create domain` | `CYCLONEDDS_URI` указывает на отсутствующий интерфейс | `unset CYCLONEDDS_URI`; убрать из `~/.bashrc` |
| В `ros2 topic list` нет `/lowstate` | Не тот интерфейс или домен, либо RMW не CycloneDDS | `dds_env.sh`; `domain_id: 0`; `echo $RMW_IMPLEMENTATION` |
| `Unknown package 'unitree_go'` | `ros2_ws` не собран или не подключён | Раздел 6; `source ros2_ws/install/setup.bash` |
| `ModuleNotFoundError` в `ros2` / `colcon` | Активно conda-окружение | `conda deactivate` |
| CMake симулятора не находит `unitree_sdk2` | SDK установлен в другой префикс | `cmake .. -DCMAKE_PREFIX_PATH=<prefix>` |