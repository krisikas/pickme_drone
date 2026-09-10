-include .env
export

.PHONY: help build-base build-base-real build-sim save-sim up-sim up-sim-d down-sim logs-sim gui gz-topics \
        build-real save-real push-save push-real push-dir sync \
        build-ros-core build-ros-camera build-ros-perception \
        restart-camera restart-core restart-fcu restart-perception \
        up-real up-real-d down-real logs-real \
        shell-dev shell-sim shell-core shell-perception shell-fcu shell-camera shell-px4

# ── Default env ──

BOARD        ?= radxa
COMPOSE_SIM   = compose/sim.compose.yaml
COMPOSE_REAL  = compose/$(BOARD).compose.yaml
DRONE_IP     ?= 192.168.1.100
DRONE_USER   ?= ubuntu

REAL_IMAGES   = pickme-base pickme-fcu pickme-camera pickme-core pickme-perception pickme-foxglove pickme-dev
SIM_IMAGES    = pickme-base pickme-sim pickme-core pickme-sim-perception pickme-perception pickme-foxglove pickme-dev

PX4_MSGS_BRANCH_SIM  ?= release/1.16
PX4_MSGS_BRANCH_REAL ?= release/1.15

# ── Help ──
help:
	@echo "=== PickmeDrone Automation & Deployment ==="
	@echo "Active Target Board: $(BOARD)"
	@echo "Drone Address:       $(DRONE_USER)@$(DRONE_IP)"
	@echo "Hardware Compose:    $(COMPOSE_REAL)"
	@echo ""
	@echo "  Live ROS 2 In-Container Development (Directly on $(BOARD)):"
	@echo "    make sync                  - Fast rsync of configs, launch files, and code to drone (1s)"
	@echo "    make build-ros-camera      - Build/symlink camera packages under root in camera container"
	@echo "    make build-ros-core        - Build C++ odometry/offboard packages under root in core container"
	@echo "    make build-ros-perception  - Build perception packages under root in perception container"
	@echo "    make restart-camera        - Restart camera container on $(BOARD) (loads edited launch/env)"
	@echo "    make restart-core          - Restart flight core container on $(BOARD)"
	@echo "    make restart-fcu           - Restart FCU bridge container on $(BOARD)"
	@echo "    make restart-perception    - Restart perception container on $(BOARD)"
	@echo ""
	@echo "  Simulation (macOS / Dev PC):"
	@echo "    make build-base-sim  - Build sim base image with px4_msgs $(PX4_MSGS_BRANCH_SIM)"
	@echo "    make build-sim        - Build simulation Docker images (depends on build-base)"
	@echo "    make up-sim           - Start PX4 SITL, Gazebo Server, ROS Bridges, and Core"
	@echo "    make up-sim-d         - Start simulation stack in background (detached)"
	@echo "    make down-sim         - Stop and remove simulation containers"
	@echo "    make logs-sim         - Follow logs from simulation containers"
	@echo "    make save-sim         - Compress and save simulation images to docker_save/sim.tar.gz"
	@echo "    make gui              - Launch Gazebo Harmonic 3D GUI on host"
	@echo "    make gz-topics        - Inspect active Gazebo transport topics"
	@echo ""
	@echo "  Full Image Build & Deploy (Heavy, only needed on new system apt packages):"
	@echo "    make build-base-real  - Build ARM64 base image with px4_msgs $(PX4_MSGS_BRANCH_REAL)"
	@echo "    make build-real       - Build all ARM64 hardware Docker images for $(BOARD)"
	@echo "    make save-real        - Compress and save hardware images to docker_save/$(BOARD).tar.gz"
	@echo "    make push-save        - Copy archive to drone, uncompress and docker load"
	@echo "    make push-real        - Full deploy pipeline (save-real + push-save)"
	@echo "    make up-real          - Start hardware stack on $(BOARD)"
	@echo "    make up-real-d        - Start hardware stack on $(BOARD) in background (detached)"
	@echo "    make down-real        - Stop hardware stack on $(BOARD)"
	@echo "    make logs-real        - Follow logs from hardware containers on $(BOARD)"
	@echo ""
	@echo "  Interactive Shells:"
	@echo "    make shell-dev        - Bash in dev container"
	@echo "    make shell-sim        - Bash in sim container"
	@echo "    make shell-core       - Bash in flight core container"
	@echo "    make shell-perception - Bash in RTAB-Map perception container"
	@echo "    make shell-fcu        - Bash in FCU bridge container (MicroXRCE + MAVLink)"
	@echo "    make shell-camera     - Bash in depth camera driver container"
	@echo "    make shell-px4        - Attach directly to PX4 SITL console"
	@echo ""

# ── Simulation ──
build-base-sim:
	docker build --build-arg PX4_MSGS_BRANCH=$(PX4_MSGS_BRANCH_SIM) -t pickme-base -f docker/base.Dockerfile .

build-sim: build-base-sim
	docker compose -f $(COMPOSE_SIM) build

save-sim:
	@mkdir -p ./docker_save
	@echo "--> Saving sim docker images to docker_save/sim.tar.gz ..."
	docker save $(SIM_IMAGES) | gzip > ./docker_save/sim.tar.gz
	@echo "--> Saved."

up-sim:
	docker compose -f $(COMPOSE_SIM) up

up-sim-d:
	docker compose -f $(COMPOSE_SIM) up -d

down-sim:
	docker compose -f $(COMPOSE_SIM) down

logs-sim:
	docker compose -f $(COMPOSE_SIM) logs -f

gui:
	./sim/run_gui.sh

gz-topics:
	@PX4_IP=$$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' pickme-sim 2>/dev/null | tr -d '[:space:]'); \
	echo "Checking Gazebo topics on $$PX4_IP (partition: sim)..."; \
	GZ_PARTITION=sim GZ_RELAY=$$PX4_IP gz topic -l

# ── Real Drone ($(BOARD)) ──
build-base-real:
	docker build --platform linux/arm64 --build-arg PX4_MSGS_BRANCH=$(PX4_MSGS_BRANCH_REAL) -t pickme-base -f docker/base.Dockerfile .

build-real: build-base-real
	DOCKER_DEFAULT_PLATFORM=linux/arm64 docker compose -f $(COMPOSE_REAL) build

save-real:
	@mkdir -p ./docker_save
	@echo "--> Saving real docker images to docker_save/$(BOARD).tar.gz ..."
	docker save $(REAL_IMAGES) | gzip > ./docker_save/$(BOARD).tar.gz
	@echo "--> Saved."

push-save:
	@echo "--> Copying to $(DRONE_USER)@$(DRONE_IP):~/pickme_drone/docker_save/$(BOARD).tar.gz ..."
	ssh $(DRONE_USER)@$(DRONE_IP) "mkdir -p ~/pickme_drone/docker_save"
	scp ./docker_save/$(BOARD).tar.gz $(DRONE_USER)@$(DRONE_IP):~/pickme_drone/docker_save/
	@echo "--> Loading images on drone..."
	ssh $(DRONE_USER)@$(DRONE_IP) "gzip -dc ~/pickme_drone/docker_save/$(BOARD).tar.gz | sudo docker load"
	@echo "--> Done."

push-real: save-real push-save

push-dir: sync

sync:
	@chmod +x docker/fcu-entrypoint.sh 2>/dev/null || true
	@mkdir -p workspace/core/{build,install,log} workspace/camera/{build,install,log} workspace/perception/{build,install,log} workspace/dev
	@echo "--> Syncing project to $(DRONE_USER)@$(DRONE_IP):~/pickme_drone/ ..."
	rsync -avz --exclude '.git' --exclude 'build' --exclude 'install' --exclude 'log' \
		--exclude '.DS_Store' --exclude 'sim' --exclude 'docker_save' \
		--exclude 'workspace/*/build' --exclude 'workspace/*/install' --exclude 'workspace/*/log' \
		--exclude 'workspace/build' --exclude 'workspace/install' --exclude 'workspace/log' \
		./ $(DRONE_USER)@$(DRONE_IP):~/pickme_drone/
	@echo "--> Project tree synced."

# ── ROS 2 In-Container Builds (Root -> Host Volumes) ──
build-ros-core:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) exec -u root core \
		bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install --packages-select pickme_odometry pickme_offboard pickme_bringup"

build-ros-camera:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) exec -u root camera \
		bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install --packages-select pickme_bringup"

build-ros-perception:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) exec -u root perception \
		bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install --packages-select pickme_bringup"

# ── Container Restarts ──
restart-camera:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) restart camera

restart-core:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) restart core

restart-fcu:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) restart fcu

restart-perception:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) restart perception

up-real:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) up

up-real-d:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) up -d

down-real:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) down

logs-real:
	sudo docker compose --env-file .env -f $(COMPOSE_REAL) logs -f

# ── Shells ──
shell-dev:
	docker compose -f $(COMPOSE_SIM) exec dev bash 2>/dev/null || docker compose -f $(COMPOSE_REAL) exec dev bash

shell-sim:
	docker compose -f $(COMPOSE_SIM) exec sim bash

shell-core:
	docker compose -f $(COMPOSE_SIM) exec core bash 2>/dev/null || docker compose -f $(COMPOSE_REAL) exec core bash

shell-perception:
	docker compose -f $(COMPOSE_SIM) exec perception bash 2>/dev/null || docker compose -f $(COMPOSE_REAL) exec perception bash

shell-fcu:
	docker compose -f $(COMPOSE_REAL) exec fcu bash

shell-camera:
	docker compose -f $(COMPOSE_REAL) exec camera bash

shell-px4:
	docker attach pickme-sim
