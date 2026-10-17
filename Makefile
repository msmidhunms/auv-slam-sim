COMPOSE := docker compose

.PHONY: help build up down ps logs shell teleop mission mission-stop mission-abort mission-status fault fault-clear gui rtf check bag test smoke smoke-down

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

teleop: ## drive the vehicle from keyboard
	$(COMPOSE) run --rm teleop

mission: ## start the survey mission
	$(COMPOSE) run --rm -T shell ros2 service call /mission/start std_srvs/srv/Trigger

mission-stop: ## stop the mission, propeller off, vehicle floats up
	$(COMPOSE) run --rm -T shell ros2 service call /mission/stop std_srvs/srv/Trigger

mission-abort: ## abort the mission, vehicle climbs to the surface
	$(COMPOSE) run --rm -T shell ros2 service call /mission/abort std_srvs/srv/Trigger

mission-status: ## current state of the mission
	@$(COMPOSE) run --rm -T shell ros2 topic echo /mission/status --once --qos-durability transient_local --field data

fault: ## inject a fault, F=leak | dvl_failure | usbl_off | battery_low
	$(COMPOSE) run --rm -T shell ros2 topic pub --times 5 --rate 2 /faults/$(F) std_msgs/msg/Bool "{data: true}"

fault-clear: ## clear a fault, F=leak | dvl_failure | usbl_off
	$(COMPOSE) run --rm -T shell ros2 topic pub --times 5 --rate 2 /faults/$(F) std_msgs/msg/Bool "{data: false}"

gui: ## open the browser desktop with RViz and Gazebo GUI
	open http://localhost:$${NOVNC_PORT:-6080}/

rtf: ## real time factor of the simulation
	@$(COMPOSE) exec -T bridge bash -c 'source /opt/ros/$$ROS_DISTRO/setup.bash && gz topic -e -t /world/underwater/stats -n 1 | grep real_time_factor'

check: ## smoke test of the running stack: topic rates, latched topics and TF
	@$(COMPOSE) run --rm -T -v $(CURDIR)/tests:/tests:ro teleop python3 /tests/stack_check.py

bag: ## record a rosbag into ./bags, Ctrl-C to stop
	$(COMPOSE) run --rm bag

test: ## unit tests of our packages
	@for pkg in auv_sensors auv_control auv_mission auv_sonar; do \
	  echo "--- $$pkg"; \
	  $(COMPOSE) run --rm -T -v $(CURDIR)/src:/ws/src:ro -w /ws/src/$$pkg shell python3 -m pytest -q -p no:cacheprovider test || exit 1; \
	done

smoke: ## talker and listener in separate containers, to check the comms
	$(COMPOSE) --profile smoke up -d
	@sleep 5
	$(COMPOSE) logs --tail 5 listener

smoke-down: ## stop the smoke test containers
	$(COMPOSE) --profile smoke down
