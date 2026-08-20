.PHONY: sim gui down build build-base logs shell-dev shell-sim shell-core help

COMPOSE_SIM  = compose/compose.sim.yaml

help:
	@echo "=== PickmeDrone Commands ==="
	@echo ""
	@echo "  Simulation (macOS):"
	@echo "    make build      - Build all simulation Docker images"
	@echo "    make build-base - Build only the base image (after changing interfaces)"
	@echo "    make sim        - Start PX4 SITL, Gazebo Server, ROS Bridges, and Core"
	@echo "    make gui        - Launch Gazebo Harmonic 3D GUI on macOS host"
	@echo "    make down       - Stop and remove all simulation containers"
	@echo "    make logs       - Tail logs from all simulation containers"
	@echo ""
	@echo "  Shells:"
	@echo "    make shell-dev  - Open bash in dev container"
	@echo "    make shell-sim  - Open bash in sim container"
	@echo "    make shell-core - Open bash in core container"

# ── Base Image ──
build-base:
	docker build -t pickme-base -f docker/base.Dockerfile .

# ── Simulation ──
build: build-base
	docker compose -f $(COMPOSE_SIM) build

sim:
	docker compose -f $(COMPOSE_SIM) up

down:
	docker compose -f $(COMPOSE_SIM) down

logs:
	docker compose -f $(COMPOSE_SIM) logs -f

gui:
	@./sim/run_gui.sh

gz-topics:
	@PX4_IP=$$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' pickme-sim 2>/dev/null | tr -d '[:space:]'); \
	echo "Checking Gazebo topics on $$PX4_IP (partition: sim)..."; \
	GZ_PARTITION=sim GZ_RELAY=$$PX4_IP gz topic -l

# ── Shells ──
shell-dev:
	docker compose -f $(COMPOSE_SIM) exec pickme-dev bash

shell-sim:
	docker compose -f $(COMPOSE_SIM) exec sim bash

shell-core:
	docker compose -f $(COMPOSE_SIM) exec pickme-core bash

shell-px4:
	docker attach pickme-sim
