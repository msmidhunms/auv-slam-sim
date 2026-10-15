# auv-slam-sim

ROS 2 Jazzy + Gazebo Harmonic simulation of a BlueROV2-style underwater vehicle with
sonar-based 3D SLAM (RTAB-Map), EKF localisation and 6-DOF thruster control.
Every node runs in its own Docker container.

> Being upgraded to a realistic torpedo survey AUV (MBARI Tethys LRAUV). The new vehicle,
> world, control, teleop, the navigation sensors (DVL with bottom lock, GPS, USBL) and the
> survey sonars (multibeam, sidescan) and the aided navigation (USBL and GPS fusion,
> bathymetric SLAM) are in place. Navigation aids, survey sonars and the mission
> state machine are being reworked phase by phase, so `make mission` is not working for
> the new vehicle as of now.

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
make check             # smoke test of the running stack
```

Tested on a MacBook with Apple M4 and Docker Desktop having 8 GB memory and 10 CPUs. The full
stack is using around 2.5 GB of memory. CPU is what matters, the Gazebo GUI alone takes
around 5 cores since all the rendering is on CPU.

All the nodes are running on simulation time. So if the `sim` container is restarted, the
other containers are also restarted by compose (`depends_on` with `restart: true`),
otherwise the EKF gets stuck when the clock goes back to zero.

## Containers

| Service | What it runs |
|---|---|
| `zenoh-router` | Zenoh router, ROS 2 discovery for all the containers |
| `sim` | Gazebo Harmonic server (headless) with the underwater world and the vehicle |
| `bridge` | `ros_gz_bridge`, topics as per `src/auv_sim/config/bridge.yaml` |
| `ahrs` | attitude and heading from the raw IMU, with compass error added |
| `dvl` | DVL model, body velocity with noise and bottom lock dropouts |
| `depth` | pressure sensor model, pressure and depth |
| `gps` | GPS model, fix only at the surface |
| `multibeam` | multibeam swath from the sonar fan |
| `swath-assembler` | bathymetry submaps from the swaths |
| `sidescan` | sidescan waterfall image from the sonar fan |
| `usbl` | USBL model, position fixes from the support ship |
| `controller` | depth, heading, attitude and velocity controller, `cmd_vel` to body wrench |
| `allocator` | thruster allocator, body wrench to 8 thrust commands |
| `localization` | local EKF, AHRS + DVL + speed model + depth to `odom -> base_link` |
| `navigation` | global EKF, same plus USBL and GPS to `map -> odom` |
| `slam` | RTAB-Map, multibeam submaps + aided navigation to `slam_map -> map` and the bathymetry map |
| `mission` | survey mission, waypoints to `cmd_vel` and setpoints for the controller |
| `eval` | estimate against ground truth, errors as topics for plotting (simulation only) |
| `state-pub` | `robot_state_publisher` with the vehicle URDF |
| `gui` | desktop in the browser (noVNC) with RViz and Gazebo GUI, port 6080 |
| `foxglove` | `foxglove_bridge` for live data in Foxglove, port 8765 |
| `teleop` | keyboard teleop, started with `make teleop` |
| `bag` | rosbag recording, started with `make bag` |
| `shell` | throwaway ROS 2 shell, started with `make shell` |
| `talker`, `listener` | comms smoke test, started with `make smoke` |

## Simulation

The vehicle is a torpedo survey AUV based on the MBARI Tethys LRAUV: 2.3 m long, 0.3 m
diameter, 148 kg, one propeller, a rudder and an elevator. Like a real AUV it cannot hover
or turn on the spot, it needs forward speed to steer and dive, and it is 0.4 kg positively
buoyant so it comes up to the surface if the propeller stops. Mass, added mass, drag and
fin coefficients are from the Gazebo `auv_controls.sdf` example.

The survey area is 300 x 300 m of sandy seabed around 40 m depth (between -46 and -34 m),
with four rock fields, a 270 m pipeline lying on the seabed and a ship wreck. All of it is
generated at build time from `src/auv_sim/scripts/` (`terrain.py` is the shared seabed
height). Water surface is at `z = 0` and the vehicle starts at the surface.

| ROS topic | Type | Direction |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim -> ROS |
| `/ground_truth/odom` | `nav_msgs/Odometry` | sim -> ROS, for sensors simulation and evaluation only |
| `/propeller/cmd_thrust` | `std_msgs/Float64` | ROS -> sim, thrust in N, positive is forward (max 20 N) |
| `/fins/rudder/cmd_angle` | `std_msgs/Float64` | ROS -> sim, rad, +-0.26, positive turns to the right |
| `/fins/elevator/cmd_angle` | `std_msgs/Float64` | ROS -> sim, rad, +-0.26, negative is nose down |

Measured open loop at the surface: 7 N gives 0.95 m/s and 15 N gives 1.37 m/s, it runs
straight with the fins at zero, and -0.1 rad of elevator at 0.8 m/s pitches it 16 deg
nose down and it dives.

Ocean current is not simulated. The hydrodynamics plugin supports it, but the LiftDrag
plugin of the fins is not taking it into account, so with a cross current the fins see a
false side slip and the vehicle keeps turning by itself.

Vehicle 3D model and textures: "MBARI Tethys LRAUV" by Mabel Zhang, from Gazebo Fuel,
licensed under [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). It is downloaded
during `make build`, so the first build needs internet.

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
setpoints / cmd_vel -> controller -> /propeller/cmd_thrust, /fins/rudder/cmd_angle, /fins/elevator/cmd_angle
```

The vehicle is under actuated (one propeller, a rudder and an elevator), so it is
controlled the way survey AUVs are normally controlled:

| Loop | How |
|---|---|
| Speed | propeller thrust = hull drag feed forward + PI on the DVL speed |
| Heading | rudder from a PID on the heading error |
| Depth | outer loop makes a pitch reference (max 25 deg) from the depth error, inner loop makes the elevator angle from the pitch error |
| Altitude | same as depth, the depth reference is made from the DVL altitude (needs the DVL altitude topic, coming in the next phase) |
| Roll | not controlled, the vehicle is stable in roll by itself |

Fin force goes with speed square, so the fin loops are gain scheduled with
(1 m/s / speed)^2. Feedback comes directly from AHRS, depth sensor and DVL (not from
localisation), so the vehicle stays under control even if localisation or SLAM is down.
Control laws are in `src/auv_control/auv_control/torpedo.py`, gains in
`src/auv_control/config/control.yaml`.

| ROS topic | Type | Meaning |
|---|---|---|
| `/speed_setpoint` | `std_msgs/Float64` | m/s, 0 .. 1.6 |
| `/heading_setpoint` | `std_msgs/Float64` | rad |
| `/depth_setpoint` | `std_msgs/Float64` | z in m, for example `-20.0`, switches to depth mode |
| `/altitude_setpoint` | `std_msgs/Float64` | m above the seabed, switches to altitude mode |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop: `linear.x` speed in m/s, `linear.z` depth rate in m/s (up is positive), `angular.z` heading rate in rad/s |
| `/controller/speed_reference`, `heading_reference`, `depth_reference`, `pitch_reference` | `std_msgs/Float64` | current references, for plotting |

If no setpoint and no `cmd_vel` is coming for 5 s, the propeller is switched off and the
vehicle comes up to the surface by its own buoyancy, which is the normal safe behaviour of
an AUV. If the IMU stops, a wall clock watchdog stops the propeller. The controller can be
switched off with `ros2 service call /controller/enable std_srvs/srv/SetBool "{data: false}"`.

Measured against ground truth:

| Test | Result |
|---|---|
| Dive from the surface to 10 m at 1 m/s | max pitch 25 deg, roll stays at 0 deg |
| Depth steps of 2 to 10 m at 1 m/s | max 0.27 m overshoot, within 0.3 m after 60 s |
| Depth hold at 0.6 m/s | within 0.55 m |
| Heading step of 90 deg | within 3 deg after 24 s, 6 deg overshoot, depth within 0.33 m |
| U turn (180 deg) | turning circle around 13 m diameter |
| Speed 0.6 / 1.0 / 1.4 m/s | 0.59 / 1.01 / 1.42 m/s |

Depth holding is the weak point: the vehicle is 0.4 kg positive, so it needs a small nose
down trim which the integral finds slowly.

### Teleop

```bash
make teleop
```

| Key | Action |
|---|---|
| `w` / `s` | speed up / down in steps of 0.2 m/s (0 .. 1.6) |
| `a` / `d` | turn left / right, the opposite key goes straight again |
| `r` / `f` | go up / down, the opposite key holds the depth again |
| `space` | propeller off, stop turning, hold depth |

The vehicle needs speed to turn and to dive. At 0 m/s it floats up slowly. The command
stays till it is changed. The Teleop panel of Foxglove also works, it publishes on `/cmd_vel`.

## Sensors

| ROS topic | Type | Rate | Source |
|---|---|---|---|
| `/sonar/points` | `sensor_msgs/PointCloud2` | 5 Hz | Gazebo, forward looking sonar for obstacle avoidance, 64 x 16 beams, 120 x 40 deg, 0.5 to 30 m |
| `/imu/raw` | `sensor_msgs/Imu` | 50 Hz | Gazebo, gyro and accel with noise, orientation is perfect |
| `/imu/data` | `sensor_msgs/Imu` | 50 Hz | `ahrs`, orientation with noise and slowly wandering heading error (1.5 deg) |
| `/dvl/twist` | `geometry_msgs/TwistWithCovarianceStamped` | 10 Hz, only with bottom lock | `dvl`, 1 cm/s noise, 0.2 % scale error, 2 % dropouts |
| `/dvl/altitude` | `sensor_msgs/Range` | 10 Hz | `dvl`, altitude from the 4 beams, `inf` without bottom lock |
| `/dvl/beams` | `sensor_msgs/PointCloud2` | 10 Hz | Gazebo, raw beam returns for the DVL node |
| `/pressure` | `sensor_msgs/FluidPressure` | 25 Hz | `depth`, absolute pressure |
| `/depth/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 25 Hz | `depth`, only z is valid, 1 cm noise |
| `/gps/fix` | `sensor_msgs/NavSatFix` | 1 Hz | `gps`, fix only at the surface, `NO_FIX` under water |
| `/gps/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 1 Hz, only with fix | `gps`, x and y in the map frame, 2.5 m noise |
| `/usbl/pose` | `geometry_msgs/PoseWithCovarianceStamped` | every 5 s | `usbl`, position from the support ship, stamped at ping time |
| `/usbl/ship` | `geometry_msgs/PointStamped` | latched | `usbl`, ship position (-20, -20, 0) |
| `/speed_model/twist` | `geometry_msgs/TwistWithCovarianceStamped` | 10 Hz | `controller`, speed from the propeller command, 0.15 m/s |

How they behave, like the real instruments:

- **DVL** has bottom lock only within 30 m slant range (a 1200 kHz unit). Four Janus beams
  at 30 deg are traced in Gazebo and at least 3 of them have to hit the seabed. The seabed
  is 34 to 46 m deep, so there is no lock at the surface, and it comes at around 14 m depth
  during the dive (measured 13.9 m). Altitude is the mean of the beams, on the dunes it
  differs from the height straight below by up to 3 m (measured mean +0.6 m).
- **GPS** works only at the surface, after 10 s there (antenna has to be out of water and
  the receiver has to get the satellites). Measured mean error 3.0 m.
- **USBL** fixes come from the support ship every 5 s, 10 % of the pings get no reply.
  Error is 0.5 % of slant range (minimum 0.2 m) per axis, and the fix comes 0.4 s plus two
  way sound travel late. Measured: 26 fixes in 140 s, 0.4 to 0.6 s late, 0.49 m mean error.
- **Speed model**: without DVL lock the only velocity is the speed worked out from the
  propeller command, like real AUVs do with the propeller RPM. It is crude. In a dive
  from the surface the EKF was 11.5 m off by the time the DVL locked (around 80 s without
  DVL). GPS before the dive and USBL fixes are there to remove this, which is the next phase.

Noise values are in `src/auv_sensors/config/sensors.yaml`. The forward sonar is a
`gpu_lidar` in Gazebo, so it gives the geometry but acoustic effects (multipath, speckle,
beam width) are not modelled.

Localisation and SLAM nodes are supposed to use only these topics.
`/ground_truth/odom` is for the sensor models and for evaluation.

## Survey sonars

Real survey AUVs carry a multibeam echosounder (depth across the track) and a sidescan
sonar (seabed image). Both are made here from one acoustic fan which is traced in Gazebo:
512 rays across the track, +-85 deg from the nadir, 10 Hz (`sonar_fan_link`, x down).

| ROS topic | Type | Rate | Meaning |
|---|---|---|---|
| `/sonar_fan/points` | `sensor_msgs/PointCloud2` | 10 Hz | raw fan from Gazebo |
| `/multibeam/swath` | `sensor_msgs/PointCloud2` | 10 Hz | `multibeam`, depth points of one ping, +-60 deg (120 deg swath) |
| `/multibeam/submap` | `sensor_msgs/PointCloud2` | every 15 m of track | `swath-assembler`, swaths put together with the dead reckoning, 0.25 m voxels, in `base_link` |
| `/sidescan/waterfall` | `sensor_msgs/Image` | 2 Hz | `sidescan`, scrolling waterfall, port on the left, newest ping on the top |

- **Multibeam**: range noise is 0.2 % at the nadir and 4 times more at the outer beams. The
  swath is 3.5 times the altitude wide (35 m at 10 m altitude).
- **Submaps**: a single swath is only a line and cannot be matched to anything, so 15 m of
  track is put together into one bathymetry submap (around 13 000 points). This is the
  input for SLAM in the next phase, same idea as submap based bathymetric SLAM.
- **Sidescan**: echo strength comes from the angle between the ray and the seabed (Lambert),
  with Rayleigh speckle. Rays hidden behind an object leave a dark acoustic shadow, and the
  water column under the vehicle is dark. 60 m range per side, 0.2 m bins, slant range (not
  corrected to ground range), like a raw sidescan record. Open it in the Image panel of
  Foxglove.

Measured on a run in altitude mode (10 m) across the pipeline and the rock field: the
pipeline shows as a bright line, rocks with their shadows, dune crests as bands. Altitude
was held at 10.5 m mean, 1.0 m std (9.0 to 12.6 m) over the dunes. 20 submaps were made.

Limits: there is no material information, so a rock and sand at the same angle give the
same echo; there is no beam pattern, multipath or surface reflection; far out on the dunes
the shadows come in blocky bands because the fan has only 512 rays.

## Navigation

Two `robot_localization` EKFs, same way as on real AUVs which combine the inertial / DVL
navigation with USBL:

| Filter | Container | Inputs | Output |
|---|---|---|---|
| local | `localization` | AHRS, DVL, propeller speed model, depth | `odom -> base_link`, `/odometry/filtered`. Continuous, but drifts |
| global | `navigation` | same plus USBL fixes and GPS (at surface) | `map -> odom`, `/odometry/global`. Absolute, drift is taken out at every fix |

USBL fixes come 0.4 to 0.6 s late with the ping time in the stamp, the global filter keeps
3 s of history and puts them in at the right time. A fix more than 5 sigma away from the
estimate is dropped. The `map` frame is the world frame of the survey (USBL and GPS give
world coordinates). Config is in `src/auv_localization/config/ekf.yaml`.

The depth sensor publishes the same depth twice, as `/depth/pose` in `odom` for the local
filter and `/depth/pose_map` in `map` for the global one. With one topic in `odom`, the
global filter had to transform it with the `map -> odom` TF which it publishes itself, and
this feedback made it run away to 35 km with a tiny covariance (found in testing).

## SLAM

RTAB-Map builds the bathymetric map from the multibeam submaps (one graph node per 15 m
submap, 35 to 40 m wide). The submap of a new node is matched with ICP against older
submaps nearby (25 m radius). Bathymetry looks the same from any heading, so the
neighbouring survey lanes, which are run in opposite direction, also match.

Odometry for RTAB-Map is the aided navigation (`map -> base_link`), so the graph is close
to the truth already, and RTAB-Map makes the map consistent where the lanes overlap. Its
correction is published as `slam_map -> map`.

| Output | Type | Meaning |
|---|---|---|
| TF `slam_map -> map` | | SLAM correction on top of the aided navigation |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | bathymetry map, 0.5 m voxels |
| `/rtabmap/mapPath` | `nav_msgs/Path` | optimised trajectory |
| `/rtabmap/mapGraph` | `rtabmap_msgs/MapGraph` | pose graph with the loop closure links |

TF tree: `world -> slam_map -> map -> odom -> base_link`. `world -> slam_map` is a static
identity, only for showing the simulator ground truth in RViz.

The vehicle navigates on `map -> base_link` (aided navigation), not on the SLAM pose. Real
survey AUVs do the same: the navigation is INS / DVL with USBL, and the bathymetry is made
consistent afterwards. Feeding the SLAM loop closures back into the navigation would need
the USBL fixes inside the SLAM graph as priors, which was not done here.

### Measured

Survey test: GPS fix at the surface, dive and transit to the survey area, then 4 lanes of
100 m, 25 m apart, at 10 m altitude and 1 m/s (526 m, 630 s, 118 USBL fixes, 36 GPS fixes).
Horizontal position error against ground truth:

| Estimate | RMS | Max | At the end |
|---|---|---|---|
| Dead reckoning (local EKF) | 4.28 m | 6.13 m | 5.06 m |
| Aided navigation (global EKF, USBL + GPS) | 0.90 m | 3.68 m | 0.34 m |
| SLAM (`slam_map -> base_link`) | 0.91 m | 3.68 m | 0.47 m |

- USBL aiding keeps the error bounded, dead reckoning keeps growing. The maximum of the
  aided navigation comes in the dive, when the DVL has no bottom lock yet and the speed
  model is the only velocity. It comes down to 0.2 to 0.4 m once the DVL locks.
- SLAM is not adding accuracy here: with these settings it made only 1 loop closure in 41
  submaps, so it stays on the aided navigation. With a wider search (35 m) and a lower
  overlap requirement it made 14 loop closures, but got worse (1.34 m rms), because the
  sparse and noisy multibeam over smooth dunes gives wrong matches. With 0.3 to 0.9 m USBL
  aiding there is not much left for SLAM to correct. It is useful where there is no USBL,
  and over rough seabed (rock fields, the wreck).

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
| `/eval/ekf_position_error` | horizontal error of the local EKF (dead reckoning), m |
| `/eval/nav_position_error` | horizontal error of the global EKF (USBL and GPS aided), m |
| `/eval/slam_position_error` | horizontal error of the SLAM pose (`slam_map -> base_link`), m |
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

## Recording

```bash
make bag      # Ctrl-C to stop
```

Sensor, control, estimate, ground truth and evaluation topics are recorded in MCAP format
into `./bags` (not tracked in git). Sonar cloud is the bulky one, a bag is around
50 MB per minute. The MCAP file can be opened directly in Foxglove.

## Tests

```bash
make check    # smoke test of the running stack: topic rates, latched topics and TF chain
make test     # unit tests of sensor models, control laws, sonar models and survey guidance
```

## Visualisation

**RViz and Gazebo GUI** - open `http://localhost:6080` (or `make gui`). RViz is on the left
and Gazebo is on the right. Give 20 to 30 seconds after `make up`, Gazebo window takes
some time to come.

**Foxglove** - in the Foxglove app (desktop or `app.foxglove.dev` in Chrome) select
Open connection -> Foxglove WebSocket -> `ws://localhost:8765`. Then import the layout from
`foxglove/layout.json` (Layouts -> Import from file). It is having the 3D view with sonar
and SLAM map, plots of position error, depth, DVL velocity and thruster commands, mission
status and a teleop panel.

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
| `make check` | smoke test of the running stack |
| `make bag` | record a rosbag into `./bags` |
| `make rtf` | real time factor of the simulation |
| `make smoke` | talker and listener in separate containers, to check the comms |
| `make smoke-down` | stop the smoke test containers |

## Middleware

All containers are on the `auv_net` compose network and discover each other through the
Zenoh router container (`rmw_zenoh_cpp`). To switch to CycloneDDS, change
`RMW_IMPLEMENTATION` in `.env` to `rmw_cyclonedds_cpp`.

## License

MIT
