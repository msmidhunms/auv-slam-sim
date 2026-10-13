# auv-slam-sim

ROS 2 Jazzy + Gazebo Harmonic simulation of a BlueROV2-style underwater vehicle with
sonar-based 3D SLAM (RTAB-Map), EKF localisation and 6-DOF thruster control.
Every node runs in its own Docker container.

> All the nodes are in place: simulation, sensors, control, teleop, EKF localisation, SLAM,
> survey mission, evaluation and visualisation. Foxglove layout and rosbag recording are pending.

## Prerequisites

- Docker Desktop with Compose v2
- `make`

ROS 2 is not required on the host machine. All `ros2` commands are to be run inside the
containers only.

## Quick start

```bash
cp .env.example .env   # optional, defaults are same as the example
make build             # build the images, first time it takes around 10 min for Gazebo
make up                # simulator, sensors, control, localisation, SLAM, gui and foxglove bridge
make ps
make gui               # opens http://localhost:6080 in the browser
make teleop            # drive the vehicle from keyboard
make mission           # or let it run the survey by its own
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
| `localization` | `robot_localization` EKF, AHRS + DVL + depth to `odom -> base_link` |
| `slam` | RTAB-Map, sonar cloud + odometry to `map -> odom` and the 3D map |
| `mission` | survey mission, waypoints to `cmd_vel` and setpoints for the controller |
| `eval` | estimate against ground truth, errors as topics for plotting (simulation only) |
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

## Localisation

`robot_localization` EKF is fusing the heading and angular rates from AHRS, body velocity
from DVL and z from the depth sensor. Config is in `src/auv_localization/config/ekf.yaml`.

| Output | Type | Meaning |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` | estimated pose and velocity, 50 Hz |
| TF `odom -> base_link` | | same estimate as TF |
| TF `world -> map` | static | only for showing the ground truth along with the estimate in RViz |

This is dead reckoning. Depth and heading are absolute, but nothing is observing x and y,
so the horizontal position drifts with the distance travelled. Correcting that drift is the
job of SLAM.

Measured against ground truth, two 15 m squares at 0.5 m/s and 5 m depth (118 m in 313 s):

| Quantity | Error |
|---|---|
| Horizontal position | max 1.06 m, 0.97 m at the end (0.8 % of distance travelled) |
| Depth | max 3.1 cm |
| Heading | mean -0.5 deg, between -2.4 and +1.2 deg (it is the compass error of the AHRS) |

The error is not growing steadily in this run because the path is a closed square, the
heading error partly cancels on opposite legs. On a long straight run it will grow more.

`world` and `map` frames are the same only when the simulator and the localisation node
are started together (`make up` does that), since the filter starts from x = y = 0.
If the `localization` container alone is restarted in between, ground truth and estimate
will be shown with an offset in RViz.

## SLAM

RTAB-Map is running in sonar only mode. The EKF dead reckoning is its odometry, a node is
added to the graph on every 0.5 m, and the sonar cloud of the new node is matched with ICP
against the clouds of nearby older nodes (proximity detection). On a successful match the
graph is optimised and the correction comes out as the `map -> odom` TF. Config is in
`src/auv_slam/config/rtabmap.yaml`.

| Output | Type | Meaning |
|---|---|---|
| TF `map -> odom` | | drift correction, so `map -> base_link` is the SLAM pose |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | 3D map, 0.25 m voxels |
| `/rtabmap/mapPath` | `nav_msgs/Path` | optimised trajectory |
| `/rtabmap/mapGraph` | `rtabmap_msgs/MapGraph` | pose graph with the loop closure links |

Measured against ground truth, two laps of a 20 m square at 0.5 m/s and 5 m depth
(163 m in 392 s). 260 graph nodes and 127 proximity links were created.

| Position error | EKF alone | SLAM |
|---|---|---|
| Back at the start after lap 1 | 0.51 m | 0.09 m |
| Back at the start after lap 2 | 0.34 m | 0.03 m |
| RMS over the full run | 0.45 m | 0.42 m |
| Maximum | 0.54 m | 0.66 m |

So SLAM is pulling the position back when the vehicle comes to an already mapped place,
but in between the revisits it is only as good as the dead reckoning. In this run the dead
reckoning itself was within 0.55 m, so the gain is small. It will matter more on longer runs.

Points to note:

- ICP between consecutive clouds is switched off (`RGBD/NeighborLinkRefining`). On open
  seabed nothing is holding x and y and ICP slides, DVL dead reckoning is better there.
- There is no camera, so appearance based loop closure is not available. The vehicle has
  to come within 8 m of an old pose (`RGBD/LocalRadius`) for a loop closure.
- RTAB-Map prints `Missing visual features ...` warnings, these are from the visual
  proximity check which has nothing to work with. They can be ignored.
- A new map is started on every start of the `slam` container. The database is kept in the
  `slam_data` volume at `/data/rtabmap.db`.

## Mission

```bash
make mission        # start the survey
make mission-stop   # stop it, vehicle holds position
make logs S=mission
```

The mission node runs a lawnmower survey over the pipeline and the rocks: 5 lanes of 30 m
with 6 m spacing at 5 m depth, 242 m in total. It takes the position from the SLAM pose
(`map -> base_link`), points the vehicle at the next waypoint and gives the heading and
depth setpoints and the surge speed to the controller. Survey area, depth and speed are in
`src/auv_mission/config/mission.yaml`.

Neighbouring lanes are run in opposite directions, so the forward looking sonar is not
seeing the same seabed again on them and no loop closure comes there. For that reason the
first lane is run once more at the end (tie line).

| ROS interface | Type | Meaning |
|---|---|---|
| `/mission/start`, `/mission/stop` | `std_srvs/Trigger` | start from the first waypoint, stop |
| `/mission/status` | `std_msgs/String` | `IDLE`, `RUNNING waypoint n/N` or `DONE` |
| `/mission/path` | `nav_msgs/Path` | planned waypoints, shown in RViz as yellow line |

When the mission is not running it does not publish any command, so teleop can be used.

## Evaluation

The `eval` node compares the estimates with the ground truth and publishes the errors, for
plotting in Foxglove. A summary is also logged every 60 s (`make logs S=eval`).

| ROS topic (`std_msgs/Float64`) | Meaning |
|---|---|
| `/eval/ekf_position_error` | horizontal error of the EKF dead reckoning, m |
| `/eval/slam_position_error` | horizontal error of the SLAM pose, m |
| `/eval/depth_error` | m |
| `/eval/heading_error_deg` | deg |
| `/eval/distance_travelled` | m |

Result of the full survey mission (249 m travelled, completed in 506 s, 369 graph nodes,
61 proximity links, all of them on the tie line):

| Position error | EKF alone | SLAM |
|---|---|---|
| RMS over the mission | 0.55 m | 0.50 m |
| Maximum | 0.81 m | 0.81 m |
| At the end | 0.73 m | 0.66 m |

The vehicle stopped 0.66 m away from the last waypoint in ground truth. Depth error at the
end was 1.7 cm and heading error was 0.4 deg.

SLAM gain is small in this mission, and it has to be understood properly. A loop closure
brings the vehicle back to where SLAM believes the first lane was, and the first lane
itself was mapped with around 0.5 m of dead reckoning error by its end. So SLAM is making
the map consistent with itself, it cannot remove the error which was already there when
the place was mapped first. To get absolute accuracy an absolute reference is needed
(USBL, or GPS fix at the surface).

## Tests

```bash
make test     # unit tests of sensor models, thruster allocation, PID and survey guidance
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

In RViz the fixed frame is `map` and the vehicle model is drawn at the SLAM pose. Coloured
boxes are the SLAM map, white points are the live sonar cloud, pink line is the SLAM
trajectory, yellow line is the mission plan, orange arrows are the EKF estimate and green
arrows are the ground truth.
The `slam` container has to be running for RViz to show anything, since it gives `map -> odom`.

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
| `make mission` | start the survey mission |
| `make mission-stop` | stop the mission, vehicle holds position |
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
