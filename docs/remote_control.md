# Remote control

The web GUI has separate **Dashboard** and **Remote Control** pages. Remote
Control is designed for a phone, tablet or laptop and keeps the camera and
essential robot feedback beside the motion controls. It reports controller
state and gait, measured base speed and yaw rate, position, the most-loaded and
hottest joints, backend/world and mapping status. The full joint and rolling
RMS diagnostics remain on the Dashboard.

## Start and connect

For local use, launch normally and open the direct page at
<http://localhost:8080/remote>. The **Remote Control** tab and browser
back/forward navigation use the same route:

```bash
ros2 launch robodog_bringup robot.launch.py backend:=mujoco world:=house
```

The server binds to loopback by default. To use a phone on the same trusted
LAN, bind it to all interfaces and open `http://<robot-ip>:8080/remote` on the phone:

```bash
ros2 launch robodog_bringup robot.launch.py \
  backend:=mujoco world:=house web_host:=0.0.0.0
```

The server requires the command WebSocket to have the GUI's same origin, but it
does not provide login, authorization or TLS. `web_host:=0.0.0.0` is therefore
for an isolated, trusted LAN. Do not expose port 8080 to the internet. Put an
authenticated TLS reverse proxy and network access control in front of it if
it must cross an untrusted network.

## Drive controls

Press **Start travel (trot)**, then hold a control to move. The remote selects
the tuned trot controller because the recorded 1 m/s walk operating point is
unstable; see [locomotion validation](locomotion_validation.md).

| Input | Command |
|---|---|
| Left touch joystick | Forward/back and lateral velocity |
| Right touch joystick | Yaw rate |
| `W` / `S` or up/down arrows | Forward/back |
| `A` / `D` | Lateral left/right |
| `Q` / `E` or left/right arrows | Turn left/right |

The speed slider starts at **0.5 m/s**. It remains bounded by the server's
backend limit: simulation allows up to 2.0 m/s, while hardware remains limited
to 0.5 m/s. Releasing touch, mouse or keys sends zero velocity immediately.
Changing page, hiding the tab or losing window focus also sends zero.

While a control is held, the browser refreshes the velocity command at 10 Hz.
The web server grants one connected operator a 0.30 s drive lease and rejects
another operator's nonzero command while that lease is active. A disconnect or
expired lease publishes zero. Independently, the 400 Hz controller zeros a
streamed body velocity after 0.35 s without a fresh `/cmd_vel` message. These
timeouts stop translation and yaw; they leave the selected gait controller
active. Use **Stand** or **STOP MOVEMENT** to select standing as well as zeroing
velocity. The red emergency-stop buttons remain the control for an actual
latched stop.

## Greeting

**Greeting** performs a short front-left-leg wave and returns to the stand
pose. It is accepted only in MuJoCo, with joints enabled, no latched emergency
stop, and the controller already at a stable stand pose. Any gait or velocity
request cancels the greeting and first returns the robot to stand. Hardware
rejects this behavior until three-leg stability has been validated on the
assembled robot.

## Save a room scan

When RTAB-Map is active, **Save room scan** creates one immutable export of the
map accumulated so far. The optional name accepts 1–64 letters, digits, dots,
underscores or dashes. The browser cannot choose a server filesystem path. By
default exports go below `~/.ros/robodog/exports`; an operator can set a fixed
server-side root with the `mapping_export_directory:=/absolute/path` launch
argument.

The equivalent CLI supports an explicit absolute destination:

```bash
ros2 run robodog_perception save_map \
  --output-dir /absolute/path/to/room-scans --name workshop
```

Each completed session directory contains:

- `cloud_map.pcd`: binary PCD v0.7 from RTAB-Map's assembled
  `/robodog/mapping/cloud_map`, including the fields supplied by that map;
- `octomap.ot`: the full colored occupancy tree for occupied, free and unknown
  3D space;
- `rtabmap.db.back`: a consistent database backup containing the SLAM graph and
  observations; and
- `manifest.json`: format/version, source metadata, point count, frame, and
  SHA-256 plus byte size for every artifact.

The manifest is written last. A failed export removes partial artifacts, and an
existing session name is never overwritten. Move through the room and wait for
mapping updates before saving; the service rejects an empty or unavailable
assembled cloud. It also rejects a map whose assembled cloud has not updated
within the preceding five seconds. The persistent live database and a saved room-scan bundle
serve different purposes: restart from the database to continue mapping, and
use PCD/OctoMap files in inspection and downstream mapping tools. See the
[mapping runbook](mapping.md) for frames, topics and validation boundaries.
