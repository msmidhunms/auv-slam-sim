# auv-slam-sim

ROS 2 Jazzy + Gazebo Harmonic simulation of a BlueROV2-style underwater vehicle with
sonar-based 3D SLAM (RTAB-Map), EKF localisation and 6-DOF thruster control.
Every node runs in its own Docker container.

> Work in progress. As of now the vehicle model, underwater world, sensors and the
> visualisation are in place. Control, localisation and SLAM nodes will be added one by one.

## Prerequisites

- Docker Desktop with Compose v2
- `make`

ROS 2 is not required on the host machine. All `ros2` commands are to be run inside the
containers only.

## Quick start

```bash
cp .env.example .env   # optional, defaults are same as the example
make build             # build the images, first time it takes around 10 min for Gazebo
make up                # simulator, bridge, sensors, state publisher, gui and foxglove bridge
make ps
make gui               # opens http://localhost:6080 in the browser
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
| `state-pub` | `robot_state_publisher` with the vehicle URDF |
| `gui` | desktop in the browser (noVNC) with RViz and Gazebo GUI, port 6080 |
| `foxglove` | `foxglove_bridge` for live data in Foxglove, port 8765 |
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
```

and inside the shell:

```bash
ros2 topic echo /ground_truth/odom --field pose.pose.position --once

# all four vertical thrusters pushing down with 10 N, vehicle should go down
for i in 5 6 7 8; do
  ros2 topic pub -r 20 /thrusters/thruster$i/cmd_thrust std_msgs/msg/Float64 "{data: -10.0}" &
done
ros2 topic echo /ground_truth/odom --field pose.pose.position.z
# Ctrl+C, then `kill %1 %2 %3 %4` to stop the thrusters
```

Points to note:

- Vehicle is slightly positively buoyant, so with thrusters off it comes up slowly
  (around 6 cm/s) and stays at the surface.
- Thrusters are open loop as of now. On surge the vehicle will start yawing and pitching
  by its own, this is expected (Munk moment from the added mass) and it will be taken care
  once the heading and attitude controller is added.

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
make test     # unit tests of the sensor models
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
