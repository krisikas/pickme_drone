.PHONY: sim gui down build build-base logs shell-dev shell-sim shell-core help

COMPOSE_SIM  = compose.sim.yaml
COMPOSE_REAL = compose/compose.real.yaml

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
	@echo ""
	@echo "  Real Drone (on board):"
	@echo "    make real       - Start all real drone containers"
	@echo "    make real-down  - Stop real drone containers"

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

# ── Shells ──
shell-dev:
	docker compose -f $(COMPOSE_SIM) exec pickme-dev bash

shell-sim:
	docker compose -f $(COMPOSE_SIM) exec sim bash

shell-core:
	docker compose -f $(COMPOSE_SIM) exec core bash

shell-px4:
	docker attach pickme-sim

# ── Real Drone ──
real: build-base
	docker compose -f $(COMPOSE_REAL) up -d

real-down:
	docker compose -f $(COMPOSE_REAL) down
