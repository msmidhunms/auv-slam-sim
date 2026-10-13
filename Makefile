COMPOSE := docker compose

.PHONY: help build up down ps logs shell rtf smoke smoke-down

help:
	@grep -E '^[a-z-]+:.*##' $(MAKEFILE_LIST) | awk -F ':.*## ' '{printf "  %-12s %s\n", $$1, $$2}'

build: ## build all the images
	$(COMPOSE) build

up: ## start the stack in background
	$(COMPOSE) up -d

down: ## stop and remove all containers
	$(COMPOSE) --profile '*' down

ps: ## container status
	$(COMPOSE) --profile '*' ps

logs: ## follow logs, use S=<service> for one service
	$(COMPOSE) --profile '*' logs -f $(S)

shell: ## open a ROS 2 shell on the network, for ros2 topic/node commands
	$(COMPOSE) run --rm shell

rtf: ## real time factor of the simulation
	@$(COMPOSE) exec -T bridge bash -c 'source /opt/ros/$$ROS_DISTRO/setup.bash && gz topic -e -t /world/underwater/stats -n 1 | grep real_time_factor'

smoke: ## talker and listener in separate containers, to check the comms
	$(COMPOSE) --profile smoke up -d
	@sleep 5
	$(COMPOSE) logs --tail 5 listener

smoke-down: ## stop the smoke test containers
	$(COMPOSE) --profile smoke down
