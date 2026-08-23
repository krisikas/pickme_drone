# PickmeDrone — Архитектура

## 1. Философия и принципы

Система строится по принципу **многоуровневой модульности** и четкого разделения зон ответственности (Single Responsibility):

| Принцип | Реализация |
|---|---|
| **Изоляция полетного ядра** | Ошибки в коде миссий (`dev`), падение веб-визуализатора (`foxglove`) или зависание алгоритмов **не роняют** полетный контур (`offboard`) |
| **Единый контракт интерфейсов** | Все типы сообщений (`.msg`), сервисов (`.srv`) и действий (`.action`) изолированы в пакете `pickme_interfaces` |
| **Sim-First подход** | Разработка и валидация на 100% ведутся в симуляторе Gazebo Harmonic + PX4 SITL. Переход на железо — это лишь подключение периферийных драйверов без изменения бизнес-логики |
| **Быстрая итерация** | Код миссий и алгоритмов монтируется через Docker Volume в `dev`-песочницу, компилируется на лету без пересборки Docker-образов |
| **Синхронность полетного контура** | Внутри `pickme_offboard` полетные классы объединены в единый C++ контекст (ноду) для исключения race conditions и задержек DDS при передаче уставок |

---

## 2. Иерархия Docker-контейнеров (Sim-First)

```
 ros:humble (Официальный базовый образ)
         │
         ▼
  ┌──────────────┐
  │ pickme-base  │  ROS 2 Humble + px4_msgs + pickme_interfaces + Eigen/TF2
  │              │  (ТОЛЬКО интерфейсы и общие системные библиотеки)
  └──────┬───────┘
         │
         ├────────────────────────────────────┬────────────────────────┐
         │  FROM pickme-base                  │  FROM pickme-base      │  FROM pickme-base
         ▼                                    ▼                        ▼
  ┌──────────────┐                     ┌──────────────┐         ┌──────────────┐
  │ pickme-core  │                     │  pickme-dev  │         │pickme-       │
  │              │                     │              │         │ foxglove     │
  │ Полетный     │                     │  Песочница   │         │ (Автостарт UI│
  │ контур:      │                     │  разработчика│         │  веб-панели  │
  │ offboard,    │                     │  (gdb, rqt,  │         │  порт 8765)  │
  │ odometry     │                     │   missions)  │         └──────────────┘
  └──────────────┘                     └──────────────┘

  ┌──────────────┐                     ┌────────────────────────┐
  │  pickme-sim  │                     │  pickme-sim-perception │
  │              │                     │                        │
  │ PX4 SITL +   │                     │  ros_gz_bridge         │
  │ Gazebo       │                     │  (RGB, Depth, Point-   │
  │ Harmonic     │                     │   cloud, /clock)       │
  └──────────────┘                     └────────────────────────┘
```

---

## 3. Зоны ответственности контейнеров

### 3.1 `pickme-base`
- **Содержимое**: ROS 2 Humble, скомпилированные интерфейсы `px4_msgs` и `pickme_interfaces`, системные библиотеки (Eigen, OpenCV, `tf2_ros`, colcon).
- **Правило**: Никакого кода нод и алгоритмов. Пересобирается только при изменении `.msg`/`.srv` контрактов.

### 3.2 `pickme-sim` & `pickme-sim-perception`
- **`pickme-sim`**: Headless сервер PX4 SITL (v1.15.4) + Gazebo Harmonic + Micro-XRCE-DDS Agent + MAVLink Router.
- **`pickme-sim-perception`**: `ros_gz_bridge` — маппит топики симуляции в стандартные ROS 2 топики сенсоров (`/camera/color/*`, `/camera/depth/*`, `/camera/depth/color/points`, `/clock`).

### 3.3 `pickme-core`
- **Содержимое**: Стабильный полетный базис — `pickme_offboard` и `pickme_odometry`, запускаемые через `pickme_bringup/launch/core.launch.py`.
- **Правило**: Работает непрерывно в фоне. Защищен от внешних падений.

### 3.4 `pickme-foxglove`
- **Содержимое**: Сервер `foxglove_bridge` (порт 8765).
- **Правило**: Автоматически стартует в `compose.sim.yaml` для мгновенной отладки в Foxglove Studio, но полностью изолирован от полетного ядра `core`.

### 3.5 `pickme-dev`
- **Содержимое**: Инструменты разработчика (`gdb`, `rqt`, `tmux`, `teleop`).
- **Правило**: Директория `workspace/` или `src/` монтируется через Docker Volume. Разработчик пишет миссии и тестирует код на лету.

---

## 4. Архитектура ROS 2 пакетов (`src/`)

```text
src/
├── pickme_interfaces/       # [Типы данных] ТОЛЬКО .srv, .action
│   ├── srv/
│   │   ├── Navigate.srv     # x, y, z, yaw, speed, frame_id, auto_arm -> success, message
│   │   ├── SetAltitude.srv  # z, frame_id -> success, message
│   │   ├── SetYaw.srv       # yaw, frame_id -> success, message
│   │   └── GetTelemetry.srv # -> telemetry data (pos, vel, orientation, status)
│   └── action/
│       └── FollowTrajectory.action
│
├── pickme_odometry/         # [Оценка состояния и TF2]
│   ├── include/pickme_odometry/
│   │   └── odometry_node.hpp
│   ├── src/
│   │   └── odometry_node.cpp # PX4 NED -> ROS ENU, динамический TF (odom->base_link, odom->body), статический TF камеры, /odom
│   └── launch/
│       └── odometry.launch.py
│
├── pickme_offboard/         # [Исполнительный полетный контур]
│   ├── include/pickme_offboard/
│   │   ├── heartbeat_manager.hpp # 10Hz Heartbeat (/fmu/in/offboard_control_mode)
│   │   ├── command_sender.hpp    # Отправка VehicleCommand, проверка VehicleCommandAck и VehicleStatus
│   │   ├── trajectory_tracker.hpp # 50Hz плавная интерполяция к цели (speed * dt) + TF navigate_target
│   │   ├── safety_guard.hpp       # Проверка NaN/Inf, Geofence, watchdog одометрии
│   │   └── offboard_node.hpp      # Оркестратор: универсальные TF2 трансформации и сервисы полета
│   ├── src/
│   │   ├── heartbeat_manager.cpp
│   │   ├── command_sender.cpp
│   │   ├── trajectory_tracker.cpp
│   │   ├── safety_guard.cpp
│   │   └── offboard_node.cpp
│   └── launch/
│       └── offboard.launch.py
│
├── pickme_missions/         # [Высокоуровневая логика миссий и Python SDK]
│   ├── pickme_missions/
│   │   ├── api.py           # Легкий Python SDK (drone.navigate, drone.takeoff, drone.land)
│   │   └── flight_test.py   # Скрипты тестовых миссий
│   └── launch/
│       └── mission.launch.py
│
└── pickme_bringup/          # [Мастер-лаунчи и конфигурация]
    ├── launch/
    │   ├── core.launch.py   # Запускает odometry.launch.py + offboard.launch.py
    │   └── foxglove.launch.py # Запуск foxglove_bridge
    └── config/
        └── params.yaml      # Все параметры системы (скорости, фреймы, лимиты, топики)
```

---

## 5. Технические детали модулей

### 5.1 Почему Setpoint Tracking делается на стороне ROS 2:
1. **Динамическая скорость под задачу**: PX4 в режиме Offboard принимает либо целевую точку `position` (и летит к ней с глобальной круизной скоростью `MPC_XY_CRUISE`), либо вектор `velocity` (и летит бесконечно без точки останова). Чтобы лететь в точку $B$ с конкретной скоростью `speed` (например, 0.3 м/с для съемки или 2.0 м/с для перелета), `TrajectoryTracker` в ROS на частоте 20–50 Гц плавно двигает уставку:
   $$\Delta \vec{S} = \vec{v}_{\text{target}} \cdot \Delta t$$
2. **Плавность движения**: Исключаются рывки моторов и запредельные ускорения при больших дистанциях до цели.
3. **Системы координат**: Автоматический пересчет смещений из системы дрона `body` (например, «1 метр вперед от носа») в мировую систему `odom`/`map`.

### 5.2 Почему Offboard организован как единая нода с внутренними C++ классами:
- **Отсутствие задержек и рассинхрона**: Heartbeat (10 Гц) и уставки Trajectory (20–50 Гц) должны синхронно передаваться в PX4. Разделение их на разные ROS-процессы создает риск ситуации, когда пульс уже идет, а уставка еще застряла в межпроцессном буфере DDS.
- **Zero-Copy и надежность**: `SafetyGuard` и `TrajectoryTracker` вызываются мгновенно без создания промежуточных топиков.

### 5.3 Изоляция Foxglove Bridge:
- Запускается в отдельном легковесном контейнере `pickme-foxglove` при старте `make sim`.
- Если веб-интерфейс визуализации упадет, переполнит вебсокет или закроется браузер — полетный контур `pickme-core` продолжает штатно выполнять миссию.

---

## 6. Рабочий процесс в симуляции

```bash
# 1. Собрать образы
make build

# 2. Запустить весь симуляционный стек (PX4 SITL + Gazebo + Core + Foxglove)
make sim

# 3. Запустить 3D GUI Gazebo на хосте macOS
make gui

# 4. Открыть Foxglove Studio на хосте (ws://localhost:8765)

# 5. Войти в dev-контейнер для запуска миссий
make shell-dev
```
