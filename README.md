# auv-slam-sim

ROS 2 Jazzy + Gazebo Harmonic simulation of a BlueROV2-style underwater vehicle with
sonar-based 3D SLAM (RTAB-Map), EKF localisation and 6-DOF thruster control.
Every node runs in its own Docker container.

> Work in progress. As of now the vehicle model, underwater world, sensors, control,
> teleop and the visualisation are in place. Localisation and SLAM nodes will be added next.

## Prerequisites

- Docker Desktop with Compose v2
- `make`

ROS 2 is not required on the host machine. All `ros2` commands are to be run inside the
containers only.

## Quick start

```bash
cp .env.example .env   # optional, defaults are same as the example
make build             # build the images, first time it takes around 10 min for Gazebo
make up                # simulator, bridge, sensors, control, gui and foxglove bridge
make ps
make gui               # opens http://localhost:6080 in the browser
make teleop            # drive the vehicle from keyboard
```

Docker Desktop is to be given around 14 GB memory and 8 CPUs (Settings -> Resources).
There is no GPU inside Docker on macOS, so all the rendering is happening on CPU.

## Containers

| Service | What it runs |
|---|---|
| `zenoh-router` | Zenoh router, ROS 2 discovery for all the containers |
| `sim` | Gazebo Harmonic server (headless) with the underwater world and the vehicle |
| `bridge` | `ros_gz_bridge`, topics as per `src/auv_sim/config/bridge.yaml` |
| `ahrs` | attitude and heading from the raw IMU, with compass error added |
| `dvl` | DVL model, body velocity with noise and bottom lock dropouts |
| `depth` | pressure sensor model, pressure and depth |
| `controller` | depth, heading, attitude and velocity controller, `cmd_vel` to body wrench |
| `allocator` | thruster allocator, body wrench to 8 thrust commands |
| `state-pub` | `robot_state_publisher` with the vehicle URDF |
| `gui` | desktop in the browser (noVNC) with RViz and Gazebo GUI, port 6080 |
| `foxglove` | `foxglove_bridge` for live data in Foxglove, port 8765 |
| `teleop` | keyboard teleop, started with `make teleop` |
| `shell` | throwaway ROS 2 shell, started with `make shell` |
| `talker`, `listener` | comms smoke test, started with `make smoke` |

## Simulation

The vehicle is a BlueROV2 Heavy style ROV with 8 thrusters (4 vectored horizontal, 4 vertical).
Thruster layout is kept in `src/auv_description/config/thrusters.yaml` and the Gazebo model
is generated from it at build time. Water surface is at `z = 0` and the seabed is around
`z = -10` with a pipeline, a wreck and some rocks on it.

| ROS topic | Type | Direction |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim -> ROS |
| `/ground_truth/odom` | `nav_msgs/Odometry` | sim -> ROS, for sensors simulation and evaluation only |
| `/thrusters/thruster<1-8>/cmd_thrust` | `std_msgs/Float64` | ROS -> sim, thrust in N (max 40) |

Checking the simulation:

```bash
make rtf      # real time factor, should be near to 1.0
make shell
ros2 topic echo /ground_truth/odom --field pose.pose.position --once
```

Vehicle is slightly positively buoyant, so if the controller is stopped it comes up slowly
(around 6 cm/s) and stays at the surface.

## Control

```
cmd_vel -> controller -> wrench_cmd -> allocator -> /thrusters/thruster<1-8>/cmd_thrust
```

The controller works like the depth hold mode of a ROV autopilot. Roll and pitch are held
level, depth and heading are held, and the commands are moving their references. It is
taking the feedback directly from AHRS, depth sensor and DVL (not from localisation), so
the vehicle stays stable even if localisation or SLAM is down.

| ROS topic | Type | Meaning |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` surge and `linear.y` sway in m/s, `linear.z` depth rate in m/s (up is positive), `angular.z` heading rate in rad/s |
| `/depth_setpoint` | `std_msgs/Float64` | absolute depth as z in m, for example `-4.0` |
| `/heading_setpoint` | `std_msgs/Float64` | absolute heading in rad |
| `/wrench_cmd` | `geometry_msgs/WrenchStamped` | controller output in `base_link` |
| `/thrusters/cmd` | `std_msgs/Float64MultiArray` | all 8 thrusts in N, for plotting |
| `/controller/depth_reference`, `/controller/heading_reference` | `std_msgs/Float64` | current references, for plotting |

If `cmd_vel` is not coming for 1 s the vehicle holds its position, and if the wrench
command is not coming for 0.5 s the allocator stops the thrusters. The controller can be
switched off with `ros2 service call /controller/enable std_srvs/srv/SetBool "{data: false}"`.

Gains are in `src/auv_control/config/control.yaml`. Pitch and yaw gains are on higher side
purposefully. The vehicle is having a destabilising (Munk) moment from the added mass which
grows with speed, without the controller it starts spinning and pitching on surge.

Measured against ground truth:

| Test | Result |
|---|---|
| Depth step of 1.2 m and 4 m | settles within 5 cm in 3.7 s and 6.1 s, 3 cm overshoot |
| Surge at 0.5 and 1.0 m/s | speed error within 2 cm/s, heading within 1.3 deg, pitch within 0.3 deg |
| Sway at 0.3 m/s | 0.31 m/s, heading within 0.7 deg |
| Station hold for 20 s | 7 cm horizontal drift, 1.3 cm in depth |

Heading is held against the AHRS, so the compass error (around 1.5 deg) remains as it is.

### Teleop

```bash
make teleop
```

| Key | Action |
|---|---|
| `w` / `s` | forward / backward |
| `a` / `d` | left / right (sway) |
| `q` / `e` | turn left / right |
| `r` / `f` | up / down |
| `space` | stop everything |
| `+` / `-` | speed up / down |

Each press moves that axis by one step (stop to moving, or moving to stop) and the command
stays till it is changed. The Teleop panel of Foxglove also works, it publishes on `/cmd_vel`.

## Sensors

| ROS topic | Type | Rate | Source |
|---|---|---|---|
| `/sonar/points` | `sensor_msgs/PointCloud2` | 5 Hz | Gazebo, 128 x 32 beams, 120 x 40 deg, 0.5 to 30 m, looking 30 deg down |
| `/imu/raw` | `sensor_msgs/Imu` | 50 Hz | Gazebo, gyro and accel with noise, orientation is perfect |
| `/imu/data` | `sensor_msgs/Imu` | 50 Hz | `ahrs`, orientation with noise and slowly wandering heading error (1.5 deg) |
| `/dvl/twist` | `geometry_msgs/TwistWithCovarianceStamped` | 10 Hz | `dvl`, 1 cm/s noise, 0.2 % scale error, 2 % dropouts |
| `/pressure` | `sensor_msgs/FluidPressure` | 25 Hz | `depth`, absolute pressure |
| `/depth/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 25 Hz | `depth`, only z is valid, 1 cm noise |

Noise values are in `src/auv_sensors/config/sensors.yaml`. Sonar is a `gpu_lidar` in Gazebo,
so it gives the geometry of a multibeam sonar but acoustic effects (multipath, speckle,
beam width) are not modelled. Beam count and rate are kept low as the rendering is on CPU,
they can be changed at the top of `src/auv_sim/models/auv/model.sdf.xacro`.

Localisation and SLAM nodes are supposed to use only these topics.
`/ground_truth/odom` is for the sensor models and for evaluation.

## Tests

```bash
make test     # unit tests of sensor models, thruster allocation and PID
```

## Visualisation

**RViz and Gazebo GUI** - open `http://localhost:6080` (or `make gui`). RViz is on the left
and Gazebo is on the right. Give 20 to 30 seconds after `make up`, Gazebo window takes
some time to come.

**Foxglove** - in the Foxglove app (desktop or `app.foxglove.dev` in Chrome) select
Open connection -> Foxglove WebSocket -> `ws://localhost:8765`. For example add a Plot panel
with `/ground_truth/odom.pose.pose.position.z` to see the depth live.

Both the ports are bound to localhost only, as there is no password on the VNC.

Gazebo GUI is the heaviest one on CPU rendering (around 5 cores). If the machine is
struggling, make `START_GZ_GUI=false` in `.env` and do `make up` again, RViz alone is light.
Desktop resolution can be changed with `VNC_RESOLUTION`.

As of now the vehicle pose in RViz is the ground truth from the simulator. It will be
replaced with the estimated pose once the localisation node is added.

## Comms smoke test

```bash
make smoke        # starts router + talker + listener and prints the listener output
```

If the comms is fine, the listener will print like this:

```
listener-1  | [INFO] [listener]: I heard: [Hello World: 4]
```

It means one container is receiving the messages published from the other container.

```bash
make ps                 # talker and listener containers should be "Up", router should be "(healthy)"
make logs S=listener    # follow the listener logs, Ctrl+C to come out
make shell              # opens a ROS 2 shell in a new container on the same network
```

Inside the shell, usual ROS 2 commands will work:

```bash
ros2 node list              # /talker and /listener should be listed
ros2 topic list             # /chatter should be there
ros2 topic echo /chatter    # live messages, Ctrl+C to stop
ros2 topic hz /chatter      # around 1 Hz
exit
```

Once testing is done, stop the containers:

```bash
make smoke-down
```

## Make targets

| Target | Description |
|---|---|
| `make build` | build all the images |
| `make up` | start the stack in background |
| `make down` | stop and remove all containers |
| `make ps` | container status |
| `make logs` | follow logs, use `S=<service>` for one service |
| `make shell` | open a ROS 2 shell on the network |
| `make teleop` | drive the vehicle from keyboard |
| `make gui` | open the browser desktop with RViz and Gazebo GUI |
| `make test` | unit tests of our packages |
| `make rtf` | real time factor of the simulation |
| `make smoke` | talker and listener in separate containers, to check the comms |
| `make smoke-down` | stop the smoke test containers |

## Middleware

All containers are on the `auv_net` compose network and discover each other through the
Zenoh router container (`rmw_zenoh_cpp`). To switch to CycloneDDS, change
`RMW_IMPLEMENTATION` in `.env` to `rmw_cyclonedds_cpp`.

## License

MIT
