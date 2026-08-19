# PickmeDrone — Архитектура

## 1. Философия

Система разделена на контейнеры **по уровню стабильности и зоне ответственности**, а не по ROS-пакетам.

| Принцип | Реализация |
|---|---|
| **Изоляция железа** | Каждый аппаратный ресурс (`/dev/ttyACM0`, `/dev/bus/usb`) пробрасывается только в тот контейнер, который с ним работает |
| **Crash-изоляция** | Падение кода миссии в `dev` не убивает связь с полётником (`px4-bridge`) и не сбрасывает SLAM (`core`) |
| **Быстрая итерация** | Dev-контейнер монтирует код через `volumes`, не требует пересборки образа при правке логики |
| **Воспроизводимость** | Все зависимости описаны в Dockerfile'ах. Никаких `docker commit`. Сгорела плата — развернул за 10 минут |
| **Смена платы** | Меняется только `base.Dockerfile` (и пути `/dev/*` в compose). Код в `src/` не трогается |

---

## 2. Иерархия Docker-образов

```
 ros:humble                      (официальный образ, поддерживает ARM64 и AMD64)
         │
         ▼
  ┌──────────────┐
  │ pickme-base  │  Собирается в CI или вручную, ~раз в месяц
  │              │  ROS 2 Humble + px4_msgs + pickme_interfaces
  │              │  + общие либы (Eigen, OpenCV, colcon)
  └──────┬───────┘
         │
         │  FROM pickme-base
         ├────────────────────────────────────┐
         │                                    │
         ▼                                    ▼
  ┌──────────────┐                   ┌──────────────────┐
  │ pickme-core  │                   │  pickme-camera   │
  │              │                   │                  │
  │ Commander,   │                   │  librealsense2   │
  │ SLAM, TF,    │                   │  realsense-ros   │
  │ EKF, фильтры │                   │                  │
  └──────────────┘                   └──────────────────┘

         │  FROM pickme-base                  FROM ubuntu:22.04
         │                                    │
         ▼                                    ▼
  ┌──────────────┐                   ┌──────────────────┐
  │  pickme-dev  │                   │ pickme-px4-bridge │
  │              │                   │                  │
  │  gdb, rqt,   │                   │ Micro-XRCE-DDS  │
  │  foxglove,   │                   │ Agent            │
  │  dev-tools   │                   │ (ROS не нужен)   │
  └──────────────┘                   └──────────────────┘
```

---

## 3. Контейнеры: зоны ответственности

### 3.1 `pickme-base`

Фундамент. Все ROS-контейнеры наследуются от него.

| | |
|---|---|
| **Dockerfile** | `docker/base.Dockerfile` |
| **Базовый образ** | `ros:humble` (поддерживает ARM64 для Radxa/Jetson/RPi) |
| **Содержимое** | ROS 2 Humble, `px4_msgs` (скомпилированы), `pickme_interfaces` (скомпилированы), Eigen, OpenCV, colcon, rosdep |
| **Пересборка** | При изменении интерфейсов или обновлении ROS-зависимостей (~раз в месяц) |
| **Запускается?** | Нет. Это только базовый образ для сборки остальных |

---

### 3.2 `pickme-px4-bridge`

Связь с полётным контроллером PX4 и наземной станцией QGroundControl.

| | |
|---|---|
| **Dockerfile** | `docker/px4-bridge.Dockerfile` |
| **Базовый образ** | `ubuntu:22.04` (ROS не нужен, минимальный размер) |
| **Содержимое** | Micro-XRCE-DDS Agent v2.4.3 |
| **Железо** | `/dev/ttyACM0` (UART/USB к полётнику) |
| **Критичность** | 🔴 Максимальная. Если этот контейнер упал — дрон потерял связь с ROS |
| **Пересборка** | Почти никогда. Только при обновлении версии DDS Agent |

> Этот контейнер **не содержит** твой код. Он только транслирует данные между PX4 (uORB) и ROS 2 (DDS).
> Благодаря изоляции: даже если `dev` или `core` упадут с segfault, дрон управляется через пульт/QGC.

---

### 3.3 `pickme-camera`

Драйвер камеры глубины Intel RealSense D435.

| | |
|---|---|
| **Dockerfile** | `docker/camera.Dockerfile` |
| **Базовый образ** | `pickme-base` |
| **Содержимое** | librealsense2 SDK, `realsense-ros` (humble branch) |
| **Железо** | `/dev/bus/usb` (USB-шина камеры) |
| **ROS-топики** | Публикует: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/depth/color/points` |
| **Критичность** | 🟡 Высокая. Без камеры нет восприятия, но дрон может безопасно зависнуть |
| **Пересборка** | При обновлении SDK RealSense или смене модели камеры |

> Если в будущем добавляется LiDAR — создаётся аналогичный `pickme-lidar` контейнер с пробросом своего `/dev/spidevX.X` или `/dev/ttyUSBX`.

---

### 3.4 `pickme-core`

Стабильные фоновые сервисы. `pickme_commander` живёт здесь.

| | |
|---|---|
| **Dockerfile** | `docker/core.Dockerfile` |
| **Базовый образ** | `pickme-base` |
| **Содержимое** | `pickme_commander` (arm, takeoff, land, offboard heartbeat). В будущем: `pickme_localization` (EKF), `pickme_slam`, `pickme_transforms` (TF) |
| **Железо** | Никакого. Работает только с ROS-топиками |
| **ROS-сервисы** | `/pickme/arm`, `/pickme/disarm`, `/pickme/takeoff`, `/pickme/land` |
| **ROS-топики** | Слушает: `/fmu/out/*`. Публикует: `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint` |
| **Критичность** | 🟠 Средняя. Отлажен и стабилен, перезапускается редко |
| **Пересборка** | При изменении commander или добавлении SLAM/EKF |

---

### 3.5 `pickme-dev`

Твоя рабочая песочница. Здесь пишется и тестируется логика миссий.

| | |
|---|---|
| **Dockerfile** | `docker/dev.Dockerfile` |
| **Базовый образ** | `pickme-base` |
| **Содержимое** | Dev-инструменты: gdb, vim/nano, rqt, tmux. Пакет `pickme_missions` пробрасывается через volume |
| **Железо** | Никакого. Работает только с ROS-топиками |
| **Критичность** | 🟢 Низкая. Падает — остальные контейнеры продолжают работать |
| **Пересборка** | При добавлении новых `apt`/`pip` зависимостей (через правку Dockerfile, не через `docker commit`!) |

---

## 4. Схема коммуникации (ROS 2 DDS)

Все контейнеры на реальном дроне используют `network_mode: host` и `ROS_DOMAIN_ID=0`.
В симуляции контейнеры общаются через Docker bridge network `ros_net`.
ROS 2 DDS автоматически обнаруживает ноды во всех контейнерах.

```
┌────────────────┐         ┌────────────────┐         ┌────────────────┐
│ pickme-px4-    │         │ pickme-camera  │         │ pickme-core    │
│ bridge         │         │                │         │                │
│                │         │ [realsense_node]│        │ [commander_node]│
│ [XRCE-DDS     │         │                │         │                │
│  Agent]        │         │                │         │                │
└───────┬────────┘         └───────┬────────┘         └──┬──────────┬──┘
        │                          │                     │          │
        │ /fmu/out/*               │ /camera/*           │ svc:     │
        │ /fmu/in/*                │                     │ /pickme/ │
        │                          │                     │ arm,land │
  ──────┴──────────────────────────┴─────────────────────┴──────────┴───── ROS 2 DDS
                                                                            (host network)
        │                          │                     │          │
        ▼                          ▼                     ▼          ▼
┌──────────────────────────────────────────────────────────────────────┐
│                          pickme-dev                                  │
│                                                                      │
│  [mission_node]     — вызывает сервисы /pickme/arm, /pickme/takeoff  │
│                     — подписан на /camera/*, /fmu/out/*               │
│                                                                      │
│  docker exec -it pickme-dev bash                                     │
│  colcon build --packages-select pickme_missions                      │
│  ros2 launch pickme_missions example.launch.py                       │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 5. Структура директорий проекта

```
pickme_drone/
│
├── docker/
│   ├── base.Dockerfile             # pickme-base (фундамент)
│   ├── px4-bridge.Dockerfile       # pickme-px4-bridge (связь с PX4)
│   ├── camera.Dockerfile           # pickme-camera (RealSense)
│   ├── core.Dockerfile             # pickme-core (commander, SLAM)
│   ├── dev.Dockerfile              # pickme-dev (песочница)
│   │
│   ├── sim.Dockerfile              # PX4 SITL + Gazebo Server (только симуляция)
│   └── sim-perception.Dockerfile   # ros_gz_bridge (только симуляция)
│
├── compose/
│   └── compose.real.yaml           # Реальный дрон
│
├── compose.sim.yaml                # Симуляция (в корне для удобства make sim)
│
├── sim/
│   ├── worlds/default.sdf
│   ├── models/                     # 3D-меши (не в git, копируются из контейнера)
│   └── run_gui.sh
│
├── src/
│   │
│   │  # ── Компилируется в pickme-base ──
│   ├── pickme_interfaces/          # .msg, .srv, .action (ТОЛЬКО определения)
│   │   └── msg/DroneState.msg
│   │
│   │  # ── Компилируется в pickme-core (COPY) ──
│   ├── pickme_commander/           # PX4 команды: arm, takeoff, land, offboard
│   │   ├── pickme_commander/
│   │   │   └── commander_node.py
│   │   └── launch/
│   │       └── commander.launch.py
│   │
│   │  # ── Монтируется в pickme-dev (volume) ──
│   ├── pickme_missions/            # Логика миссий (ворота, патрулирование, посадка)
│   │   ├── pickme_missions/
│   │   │   └── example_mission.py
│   │   └── launch/
│   │       └── example.launch.py
│   │
│   └── pickme_bringup/             # Общие launch-файлы и конфиги
│
├── Makefile
├── ARCHITECTURE.md                 # Этот файл
├── README.md
└── PROJECT.md
```

**Правило распределения кода по контейнерам:**
- `pickme_interfaces` → компилируется внутри `pickme-base` (через `COPY` + `colcon build`)
- `pickme_commander` → компилируется внутри `pickme-core` (через `COPY` + `colcon build`)
- `pickme_missions` → **НЕ копируется**. Монтируется в `pickme-dev` через `volumes`. Компилируется вручную внутри контейнера

---

## 6. Рабочий процесс разработчика

### Добавление новой apt/pip зависимости

```
1. Открой docker/dev.Dockerfile
2. Допиши в QUICK-ADD SECTION:
     RUN apt-get update && apt-get install -y ros-humble-foxglove-bridge
3. На плате выполни:
     docker compose -f compose/compose.real.yaml build dev
     docker compose -f compose/compose.real.yaml up -d dev
4. Готово. Foxglove установлен навсегда, зафиксирован в Git.
```

### Добавление новой миссии

```
1. Создай файл src/pickme_missions/pickme_missions/new_mission.py
2. Создай launch-файл src/pickme_missions/launch/new_mission.launch.py
3. Зайди в dev-контейнер: docker exec -it pickme-dev bash
4. Собери: colcon build --packages-select pickme_missions
5. Запусти: ros2 launch pickme_missions new_mission.launch.py
```

### Добавление нового сенсора (например LiDAR)

```
1. Создай docker/lidar.Dockerfile (FROM pickme-base + SDK лидара)
2. В compose.real.yaml добавь сервис:
     lidar:
       image: pickme-lidar:latest
       devices: ["/dev/ttyUSB0:/dev/ttyUSB0"]
       network_mode: host
3. Лидар публикует /scan — твой код в dev подписывается на /scan
4. Существующие контейнеры не меняются
```

### Смена вычислительной платы

```
1. Отредактируй docker/base.Dockerfile под новую архитектуру
2. В compose/.env поправь пути к устройствам если изменились
3. Пересобери: docker compose build
4. Код в src/ не трогаешь
```
