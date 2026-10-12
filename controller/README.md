# Kenny main controller

This is the long-running robot process. It loads and warms the PPO checkpoint
once, keeps SLAM and sensor/pose subscriptions alive, and changes the current
destination when calls arrive. It does not spawn a model or reset localization
for each request.

```text
editor environment.json ── estimated prior ───────────────┐
                                                        ↓
LiDAR + encoder odometry → SLAM Toolbox → refined /kenny/map
                                  ↓                     ↓
                            live pose                 planner
                                  ↓                     ↓
RGB ArUco observations → corrected landmarks → resident PPO
cloud queue → nearest-call scheduler → destination       ↓
                                        measured-sensor guard
                                                        ↓
                                  /kenny/cmd_vel + motion_allowed
                                                        ↓
                              KENNY_USB_V1 motor/encoder bridge
```

## Estimated map and continuous rebuilding

- The editor map is a prior for **unobserved** areas. Its geometry is not loaded
  into SLAM as fake scan evidence or treated as a serialized SLAM pose graph.
- SLAM Toolbox continuously scan-matches LiDAR using measured wheel odometry,
  updates `slam_map -> odom`, and performs loop closure. The controller aligns
  the first live robot pose with the approximate starting pose you provide in
  the editor's `map` frame. It publishes `map -> slam_map` once.
- Every complete SLAM occupancy snapshot is transformed and resampled into the
  editor frame. Observed free space clears misplaced estimated walls; observed
  obstacles replace estimated free space. Unobserved cells retain the prior.
  Observations outside the initial room grow the map. Unknown space outside
  the prior remains unknown. Intermediate-confidence observed occupancy is
  blocked for planning until it becomes classified.
- Each new SLAM snapshot replaces the previous evidence layer, so a loop closure
  does not leave old measured walls permanently stamped into the map. Editor
  **no-go areas** remain restrictions even if LiDAR sees free floor there.
- Calibrated RGB ArUco detections refine marker locations/orientations after
  at least three consistent observations. Goals use the robot's local marker
  database, not the stale coordinates attached to a cloud call. Wall markers
  retain the 0.5 m approach offset. The planner updates after geometry or
  destination changes and at least once per second while navigating.

The map is saved every ten seconds and on orderly shutdown to
`data/refined-environment.json`. It includes corrected occupancy, updated
landmarks, the current robot start position, and observed/prior coverage counts.
Old editor walls are removed from its obstacle list so importing it does not
re-stamp them over corrected occupancy. The original estimate is not overwritten.
You can open the refined file in the editor, or use it as the next session's
prior. Supply the actual starting pose each session; saving a raster does not
restore SLAM's optimization graph or establish the robot's current location.

To retain SLAM's graph as well, use SLAM Toolbox's
`/slam_toolbox/serialize_map` service while it is running. Its graph loading
configuration is separate from the approximate-map fusion described here.
Repeated travel improves measured coverage; it does not guarantee monotonically
improving accuracy when calibration, scan matching or odometry is poor.

## Mobile live position

A separate publisher sends fresh map-frame pose estimates to Render once per
second through `/api/pi/pose`, independently of model inference and mission
switching. It includes heading, uncertainty when available, the current goal,
and a bounded route. The mobile app reads `/api/map` and `/api/robot`, draws
Kenny on the uploaded map, and marks an estimate stale after five seconds.
Missing/stale TF is not republished as a new observation.

The publisher pins the active cloud map hash. If the operator replaces that map,
the server clears the old position and rejects updates using the old hash. Fetch
the updated environment from the editor's Render panel, restart the controller
with it and the correct physical starting pose, then tracking resumes in that
map frame. Simulation playback can publish separately labeled simulation poses
for mobile UI testing; it does not establish the physical robot's location.

## Nearest-call scheduling

The controller polls the Pi queue once per second independently of the policy
control timer. When idle, it chooses the nearest registered pending destination.
During travel it compares each new pending call with the remaining straight-line
distance to the current destination from the **current live pose**.

If a pending call is closer by more than `--switch-margin` (default **0.25 m**):

1. Clear the current navigation goal and send zero motion.
2. Confirm stopping with three fresh measured-odometry samples.
3. Call `/api/pi/switch` to atomically return the current call to `queued` and
   claim the nearer one.
4. Change the resident policy's destination. The model, sensors, map and
   localization continue running.

The interrupted call is served later; it is not marked successful or discarded.
Equal/near-equal distances retain the current trip. This ranking uses direct
distance, not route length around walls. With a continuous stream of nearer calls,
farther calls can wait longer. Only one cloud call is running at a time.

The journal is `data/main-controller.json`. Lost selected-claim/switch responses
are retried idempotently; result-upload failures do not repeat navigation.
A process restart with a `running` or `switching` journal refuses automatic
physical replay. Stop the robot, inspect `/api/pi/queue` to identify the actual
running job, report it failed using the recorded worker, then remove the journal.

## Start on this workstation / Pi

Export your approximate map from the main editor and unpack `environment.json`.
Keep `Kenny-Api` beside `Kenny---Automotive-self-driving-trash-can` so the
checkpoint's configuration and feature contract can be imported. The API web
server still only needs its existing requirements; controller dependencies run
on the Pi.

The current simulator virtual environment already provides Torch, PPO, NumPy,
OpenCV and PySerial. For a fresh environment, install ROS 2 Jazzy, SLAM Toolbox,
`cv_bridge`, `sensor_msgs_py`, and the simulator's training dependencies, then
install `controller/requirements.txt`. Source ROS before using the Python venv.

Run the existing LiDAR and Astra drivers, and publish the **measured**
`base_link -> laser_frame` and `base_link -> camera optical frame` transforms.
The controller uses timestamped TF for all sensor projections. No guessed
sensor extrinsics are installed by this launch.

From `Kenny-Api`:

```sh
source /opt/ros/jazzy/setup.bash
source ~/kenny_ws/install/setup.bash
source ~/astra_ws/install/setup.bash

ros2 launch "$PWD/controller/bringup.launch.py" \
  environment:=/absolute/path/environment.json \
  model:="$HOME/Kenny---Automotive-self-driving-trash-can/releases/dynamic-v3-seed2/model/best.zip" \
  initial_x:=1.0 initial_y:=1.0 initial_yaw:=0.0
```

Replace the starting x/y/yaw with the robot's physical starting pose in the
estimated map (meters/radians). The launch keeps SLAM, the policy/controller,
and RGB landmark detection running. It expects measured `/odom` from your base.
Do not run AMCL or another publisher of `slam_map -> odom` alongside SLAM.

If using this repository's USB encoder/base adapter, append:

```sh
  motors:=true motor_port:=/dev/serial/by-id/YOUR_ESP32_MOTOR_DEVICE
```

The adapter targets the existing `joystick` repository's `KENNY_USB_V1`
firmware, telemetry order **LF, RF, LR, RR**, and provisional wheel radius
0.04 m / track 0.235 m. For different measured dimensions run
`python -m controller.motors --port DEVICE --wheel-radius R --track T`
separately. Close the browser USB bridge before using the same serial device.
It requires enabled, working encoder feedback; it will not manufacture odometry
from PWM or from the commanded velocity. ESP32 reboot/discontinuous counters
halt the base connection so localization cannot silently continue after a reset.

The installed direct-GPIO firmware currently defaults to encoders disabled.
Enable them only with the actual connected encoder wiring and calibrated counts.
The bridge publishes zero/no-motion until fresh controller permission and a
fresh bounded command are both present; the firmware watchdog remains active.

### Input and output interfaces

| Interface | Meaning |
|---|---|
| `/scan` — `LaserScan` | Fresh measured LiDAR |
| `/map` — `OccupancyGrid`, frame `slam_map` | SLAM Toolbox's complete current map |
| `/pose` — `PoseWithCovarianceStamped` | SLAM pose covariance |
| `/odom` — `Odometry`, `odom -> base_link` TF | Measured encoder/base motion |
| `/camera/depth/points` — `PointCloud2` | Measured depth points, projected using TF |
| `/camera/color/image_raw`, `/camera/color/camera_info` | RGB image and matching calibrated intrinsics |
| `/kenny/downward` — `Float32MultiArray` | `[hazard_left, hazard_center, hazard_right, valid_left, valid_center, valid_right]`, each value 0/1 |
| `/kenny/marker_observations` — `MarkerArray`, frame `slam_map` | Calibrated landmark poses from the supplied RGB detector |
| `/kenny/map` — `OccupancyGrid`, frame `map` | Refined map for the editor/planner |
| `/kenny/route` — `Path`, frame `map` | Current route |
| `/kenny/controller/status` — `String` JSON | Readiness, waiting/running status, goal, guard, coverage, resident-model flag |
| `/kenny/cmd_vel` — `TwistStamped` | Guarded forward/turn commands |
| `/kenny/motion_allowed` — `Bool` | Fresh companion permission for the motor bridge |

The live feature adapter matches the checkpoint's 269-feature frame and
four-frame history, including the actual 0.5/1/2 m route lookahead. Missing,
stale, or unobserved sensor samples are invalid, not fabricated clear ranges.
Depth projects physical obstacle-height points and measured ground/drop points;
it does not assume RGB/depth registration. The policy is held at zero until
pose/map/odometry, depth/ground coverage, and all three downward channels are
available. The existing checkpoint was trained with those downward channels;
their physical sensor publisher is a required hardware input, not supplied by
the current Astra/LiDAR drivers. Status reports which input is missing.

The default physical command cap is **0.10 m/s**, **0.60 rad/s**. Arrival uses
the live pose within 0.25 m and 0.25 rad, followed by confirmed stopping before
success is reported. Model inference and polling still take time; keeping the
policy resident removes per-call loading, not all sensor/network latency.

### See and capture the rebuilt map in the main editor

In another terminal, with ROS sourced:

```sh
python3 ~/kenny-map-creator/ros/bridge.py \
  --map-topic /kenny/map --map-frame map --port 8001
```

Open `http://localhost:8001`, connect the live-map panel to
`ws://localhost:8001/api/live`, and use **Capture map for editing** to save/export
a measured snapshot. Keep the editor's **simulation API receiver disconnected**
while the physical main controller owns the same cloud queue.

## API deployment

Deploy the updated `Kenny-Api` server before using the main scheduler:

- `GET /api/pi/queue` lists queued/running calls using the Pi role token.
- `POST /api/pi/claim` accepts an optional `job_id` for nearest-first selection.
- `POST /api/pi/switch` accepts `{worker, current_job_id, next_job_id}`.

The default URL is `https://kenny-api.onrender.com`; `KENNY_API_URL`/`--api`
can override it. Set `KENNY_PI_TOKEN` if required by the server. The scanner
submission format is unchanged; its IDs still need registration in the API.

## Verification

```sh
# API tests, using an environment with requirements-dev.txt installed:
python -m unittest discover -s tests -v

# Controller tests with ROS sourced and the simulator Python:
source /opt/ros/jazzy/setup.bash
../Kenny---Automotive-self-driving-trash-can/.venv/bin/python \
  -m unittest discover -s controller/tests -v
```

Tests cover measured geometry replacing the prior, map-frame alignment/growth,
loop-closure replacement, no-go preservation, landmark updates, blocked routes,
nearest-call interruption/resumption, lost HTTP responses, actual encoder counter
wrap, checkpoint feature equality, resident policy reuse, and ROS message/map/
stop integration with synthetic inputs. These are software checks; this change
has not run autonomous travel on the physical robot.
